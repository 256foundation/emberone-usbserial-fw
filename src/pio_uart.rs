use core::convert::Infallible;

use embassy_futures::select::{select3, Either3};
use embassy_rp::{
    clocks::clk_sys_freq,
    gpio::Level,
    peripherals::PIO1,
    pio::{Common, Config, Direction as PioDirection, FifoJoin, PioPin, ShiftDirection, StateMachine},
    usb::{self},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};
use pio_proc::pio_asm;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender},
    driver::EndpointError,
};
use embedded_io_async::{ErrorType, Write};
use fixed::traits::ToFixed;

pub enum PioUartTaskError {
    Disconnected,
}

impl From<EndpointError> for PioUartTaskError {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => PioUartTaskError::Disconnected {},
        }
    }
}

// Channel for RX data - sized to hold many packets
static RX_CHANNEL: Channel<CriticalSectionRawMutex, u8, 1024> = Channel::new();
// Signal to notify baudrate changes
static BAUD_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

/// PIO-based UART TX with baudrate change support
pub struct PioUartTx<'d> {
    sm: StateMachine<'d, PIO1, 0>,
}

impl<'d> PioUartTx<'d> {
    pub fn new(
        common: &mut Common<'d, PIO1>,
        mut sm: StateMachine<'d, PIO1, 0>,
        tx_pin: impl PioPin,
        baud: u32,
    ) -> Self {
        let tx_pin = common.make_pio_pin(tx_pin);

        let prg = pio_asm!(
            r#"
                .side_set 1 opt

                ; An 8n1 UART transmit program.
                ; OUT pin 0 and side-set pin 0 are both mapped to UART TX pin.

                    pull       side 1 [7]  ; Assert stop bit, or stall with line in idle state
                    set x, 7   side 0 [7]  ; Preload bit counter, assert start bit for 8 clocks
                bitloop:                   ; This loop will run 8 times (8n1 UART)
                    out pins, 1            ; Shift 1 bit from OSR to the first OUT pin
                    jmp x-- bitloop   [6]  ; Each loop iteration is 8 cycles.
            "#
        );
        let prg = common.load_program(&prg.program);

        let mut cfg = Config::default();
        cfg.set_out_pins(&[&tx_pin]);
        cfg.set_set_pins(&[&tx_pin]);
        cfg.use_program(&prg, &[&tx_pin]);
        cfg.shift_out.auto_fill = false;
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.fifo_join = FifoJoin::TxOnly;
        
        let clk_freq = clk_sys_freq();
        defmt::println!("PIO TX: clk_sys_freq = {}, baud = {}", clk_freq, baud);
        cfg.clock_divider = (clk_freq / (8 * baud)).to_fixed();
        
        sm.set_config(&cfg);
        
        // Set pin direction and initial level AFTER config
        sm.set_pin_dirs(PioDirection::Out, &[&tx_pin]);
        sm.set_pins(Level::High, &[&tx_pin]);
        
        sm.set_enable(true);
        defmt::println!("PIO TX: pin configured, SM enabled");

        Self { sm }
    }

    pub fn set_baudrate(&mut self, baud: u32) {
        let clock_divider = (clk_sys_freq() / (8 * baud)).to_fixed();
        self.sm.set_enable(false);
        self.sm.set_clock_divider(clock_divider);
        self.sm.set_enable(true);
    }

    pub async fn write_u8(&mut self, data: u8) {
        self.sm.tx().wait_push(data as u32).await;
    }
}

impl ErrorType for PioUartTx<'_> {
    type Error = Infallible;
}

impl Write for PioUartTx<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        for byte in buf {
            self.write_u8(*byte).await;
        }
        Ok(buf.len())
    }
}

/// PIO-based UART RX with baudrate change support
pub struct PioUartRx<'d> {
    sm: StateMachine<'d, PIO1, 1>,
}

impl<'d> PioUartRx<'d> {
    pub fn new(
        common: &mut Common<'d, PIO1>,
        mut sm: StateMachine<'d, PIO1, 1>,
        rx_pin: impl PioPin,
        baud: u32,
    ) -> Self {
        let prg = pio_asm!(
            r#"
                ; Slightly more fleshed-out 8n1 UART receiver which handles framing errors and
                ; break conditions more gracefully.
                ; IN pin 0 and JMP pin are both mapped to the GPIO used as UART RX.

                start:
                    wait 0 pin 0        ; Stall until start bit is asserted
                    set x, 7    [10]    ; Preload bit counter, then delay until halfway through
                rx_bitloop:             ; the first data bit (12 cycles incl wait, set).
                    in pins, 1          ; Shift data bit into ISR
                    jmp x-- rx_bitloop [6] ; Loop 8 times, each loop iteration is 8 cycles
                    jmp pin good_rx_stop   ; Check stop bit (should be high)

                    irq 4 rel           ; Either a framing error or a break. Set a sticky flag,
                    wait 1 pin 0        ; and wait for line to return to idle state.
                    jmp start           ; Don't push data if we didn't see good framing.

                good_rx_stop:           ; No delay before returning to start; a little slack is
                    in null 24
                    push                ; important in case the TX clock is slightly too fast.
            "#
        );
        let prg = common.load_program(&prg.program);

        let mut cfg = Config::default();
        cfg.use_program(&prg, &[]);

        let rx_pin = common.make_pio_pin(rx_pin);
        sm.set_pins(Level::High, &[&rx_pin]);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.set_jmp_pin(&rx_pin);
        sm.set_pin_dirs(PioDirection::In, &[&rx_pin]);

        cfg.clock_divider = (clk_sys_freq() / (8 * baud)).to_fixed();
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::RxOnly;
        sm.set_config(&cfg);
        sm.set_enable(true);

        Self { sm }
    }

    pub fn set_baudrate(&mut self, baud: u32) {
        let clock_divider = (clk_sys_freq() / (8 * baud)).to_fixed();
        self.sm.set_enable(false);
        self.sm.set_clock_divider(clock_divider);
        self.sm.set_enable(true);
    }

    pub async fn read_u8(&mut self) -> u8 {
        self.sm.rx().wait_pull().await as u8
    }
}

/// Dedicated task to drain PIO RX FIFO into channel buffer
/// This task runs with high priority and does nothing but read bytes
#[embassy_executor::task]
pub async fn pio_rx_task(mut rx: PioUartRx<'static>) -> ! {
    let sender = RX_CHANNEL.sender();
    let mut current_baud = 115200u32;
    
    loop {
        // Check for baudrate change signal (non-blocking)
        if let Some(new_baud) = BAUD_SIGNAL.try_take() {
            if new_baud != 0 && new_baud != current_baud {
                rx.set_baudrate(new_baud);
                current_baud = new_baud;
                defmt::println!("PIO RX baudrate set to {}", new_baud);
            }
        }
        
        // Wait for a byte from PIO and immediately send to channel
        let byte = rx.read_u8().await;
        // Use try_send to avoid blocking - if channel is full, drop the byte
        // (better than blocking and losing more bytes from PIO FIFO)
        let _ = sender.try_send(byte);
    }
}

/// USB task that handles TX and receives from channel
#[embassy_executor::task]
pub async fn usb_task2(class: CdcAcmClass<'static, super::UsbDriver>, mut tx: PioUartTx<'static>) -> ! {
    let (mut usb_tx, mut usb_rx, mut ctrl) = class.split_with_control();
    let rx_receiver = RX_CHANNEL.receiver();
    let mut current_baud = 115200u32;

    loop {
        usb_rx.wait_connection().await;
        let _ = pipe_pio_uart(&mut usb_tx, &mut usb_rx, &mut ctrl, &mut tx, &rx_receiver, &mut current_baud).await;
    }
}

/// Handle PIO UART <-> USB TTY forwarding and baudrate changes
pub async fn pipe_pio_uart<'d, T: usb::Instance + 'd>(
    usb_tx: &mut Sender<'d, usb::Driver<'d, T>>,
    usb_rx: &mut Receiver<'d, usb::Driver<'d, T>>,
    ctrl: &mut ControlChanged<'d>,
    uart_tx: &mut PioUartTx<'d>,
    rx_receiver: &embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, u8, 1024>,
    current_baud: &mut u32,
) -> Result<(), PioUartTaskError> {
    let mut usb_buf = [0; 64];
    let mut uart_buf = [0; 64];

    loop {
        let usb_read = usb_rx.read_packet(&mut usb_buf);
        let control_change = ctrl.control_changed();
        
        // Wait for first byte from channel
        let channel_read = rx_receiver.receive();

        match select3(usb_read, channel_read, control_change).await {
            // Forward data from the USB host to the UART
            Either3::First(n) => {
                let data = &usb_buf[..n?];
                let _ = uart_tx.write(data).await;
            }
            // Got first byte from channel - drain more if available
            Either3::Second(first_byte) => {
                uart_buf[0] = first_byte;
                let mut count = 1;
                
                // Drain any additional bytes from channel without blocking
                while count < uart_buf.len() {
                    match rx_receiver.try_receive() {
                        Ok(byte) => {
                            uart_buf[count] = byte;
                            count += 1;
                        }
                        Err(_) => break,
                    }
                }
                
                let data = &uart_buf[..count];
                usb_tx.write_packet(data).await?;
            }
            // Handle baudrate changes from USB CDC control requests
            Either3::Third(()) => {
                let line_coding = usb_rx.line_coding();
                let new_baud = line_coding.data_rate();
                if new_baud != 0 && new_baud != *current_baud {
                    // Update TX directly
                    uart_tx.set_baudrate(new_baud);
                    // Signal RX task to update baudrate
                    BAUD_SIGNAL.signal(new_baud);
                    *current_baud = new_baud;
                    defmt::println!("PIO UART TX baudrate set to {}", new_baud);
                }
            }
        }
    }
}

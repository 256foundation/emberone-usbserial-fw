use core::convert::Infallible;

use embassy_futures::select::{select3, Either3};
use embassy_rp::{
    clocks::clk_sys_freq,
    gpio::Level,
    peripherals::PIO1,
    pio::{Common, Config, Direction as PioDirection, FifoJoin, PioPin, ShiftDirection, StateMachine},
    usb::{self},
};
use pio_proc::pio_asm;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender},
    driver::EndpointError,
};
use embedded_io_async::{ErrorType, Read, Write};
use fixed::traits::ToFixed;
use fixed::types::U56F8;

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
        sm.set_pins(Level::High, &[&tx_pin]);
        sm.set_pin_dirs(PioDirection::Out, &[&tx_pin]);

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
        cfg.use_program(&prg, &[&tx_pin]);
        cfg.shift_out.auto_fill = false;
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.clock_divider = (U56F8::from_num(clk_sys_freq()) / U56F8::from_num(8 * baud)).to_fixed();
        sm.set_config(&cfg);
        sm.set_enable(true);

        Self { sm }
    }

    pub fn set_baudrate(&mut self, baud: u32) {
        let clock_divider = (U56F8::from_num(clk_sys_freq()) / U56F8::from_num(8 * baud)).to_fixed();
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

        cfg.clock_divider = (U56F8::from_num(clk_sys_freq()) / U56F8::from_num(8 * baud)).to_fixed();
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::RxOnly;
        sm.set_config(&cfg);
        sm.set_enable(true);

        Self { sm }
    }

    pub fn set_baudrate(&mut self, baud: u32) {
        let clock_divider = (U56F8::from_num(clk_sys_freq()) / U56F8::from_num(8 * baud)).to_fixed();
        self.sm.set_enable(false);
        self.sm.set_clock_divider(clock_divider);
        self.sm.set_enable(true);
    }

    pub async fn read_u8(&mut self) -> u8 {
        self.sm.rx().wait_pull().await as u8
    }
}

impl ErrorType for PioUartRx<'_> {
    type Error = Infallible;
}

impl Read for PioUartRx<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
        let data = self.read_u8().await;
        buf[0] = data;
        Ok(1)
    }
}

/// PIO-based UART that wraps both TX and RX with baudrate change support
pub struct PioUart<'d> {
    pub tx: PioUartTx<'d>,
    pub rx: PioUartRx<'d>,
    current_baud: u32,
}

impl<'d> PioUart<'d> {
    pub fn new(
        common: &mut Common<'d, PIO1>,
        sm_tx: StateMachine<'d, PIO1, 0>,
        sm_rx: StateMachine<'d, PIO1, 1>,
        tx_pin: impl PioPin,
        rx_pin: impl PioPin,
        baud: u32,
    ) -> Self {
        let tx = PioUartTx::new(common, sm_tx, tx_pin, baud);
        let rx = PioUartRx::new(common, sm_rx, rx_pin, baud);
        
        Self {
            tx,
            rx,
            current_baud: baud,
        }
    }

    /// Set the baudrate for both TX and RX state machines
    pub fn set_baudrate(&mut self, baud: u32) {
        if baud == 0 || baud == self.current_baud {
            return;
        }
        
        self.tx.set_baudrate(baud);
        self.rx.set_baudrate(baud);
        
        self.current_baud = baud;
        defmt::println!("PIO UART baudrate set to {}", baud);
    }
}

#[embassy_executor::task]
pub async fn usb_task2(class: CdcAcmClass<'static, super::UsbDriver>, mut uart: PioUart<'static>) -> ! {
    let (mut tx, mut rx, mut ctrl) = class.split_with_control();

    loop {
        rx.wait_connection().await;
        let _ = pipe_pio_uart(&mut tx, &mut rx, &mut ctrl, &mut uart).await;
    }
}

/// Handle PIO UART <-> USB TTY forwarding and baudrate changes
pub async fn pipe_pio_uart<'d, T: usb::Instance + 'd>(
    usb_tx: &mut Sender<'d, usb::Driver<'d, T>>,
    usb_rx: &mut Receiver<'d, usb::Driver<'d, T>>,
    ctrl: &mut ControlChanged<'d>,
    uart: &mut PioUart<'d>,
) -> Result<(), PioUartTaskError> {
    let mut usb_buf = [0; 64];
    let mut uart_buf = [0; 64];

    loop {
        let usb_read = usb_rx.read_packet(&mut usb_buf);
        let uart_read = uart.rx.read(&mut uart_buf);
        let control_change = ctrl.control_changed();

        match select3(usb_read, uart_read, control_change).await {
            // Forward data from the USB host to the UART
            Either3::First(n) => {
                let data = &usb_buf[..n?];
                let _ = uart.tx.write(data).await;
            }
            // Forward data from the UART back to the USB host
            Either3::Second(result) => {
                match result {
                    Ok(n) => {
                        let data = &uart_buf[..n];
                        usb_tx.write_packet(data).await?;
                    }
                    Err(_) => {
                        // UART read error, continue
                    }
                }
            }
            // Handle baudrate changes from USB CDC control requests
            Either3::Third(()) => {
                let line_coding = usb_rx.line_coding();
                let new_baud = line_coding.data_rate();
                uart.set_baudrate(new_baud);
            }
        }
    }
}

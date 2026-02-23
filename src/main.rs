#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    flash::{self},
    gpio::{self},
    i2c::{self},
    peripherals::{PIO0, PIO1, UART0, UART1, USB},
    pio::{self},
    pwm::{self},
    usb::{self},
    Peripheral,
};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use static_cell::StaticCell;

mod control;
mod pio_uart;
mod uart;

pub type AsicUart = UART0;
pub type AsicUart1 = UART1;
pub type I2cPeripheral = embassy_rp::peripherals::I2C1;
pub type I2cDriver = i2c::I2c<'static, I2cPeripheral, i2c::Async>;
pub type UsbPeripheral = embassy_rp::peripherals::USB;
pub type UsbDriver = usb::Driver<'static, UsbPeripheral>;
pub type UsbDevice = embassy_usb::UsbDevice<'static, UsbDriver>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => embassy_rp::uart::BufferedInterruptHandler<UART1>;
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C1>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

const FLASH_SIZE: usize = 4 * 1024 * 1024;
const VERSION: u16 = 0x0001;

static MANUFACTURER: &str = "256F";
static PRODUCT: &str = "bitcrane-S19jpro";

/// Return a unique serial number for this device by hashing its flash JEDEC ID.
fn serial_number() -> &'static str {
    let p = unsafe { embassy_rp::Peripherals::steal() };
    let flash = unsafe { p.FLASH.clone_unchecked() };
    let mut flash = flash::Flash::<_, flash::Async, FLASH_SIZE>::new(flash, p.DMA_CH0);
    static SERIAL_NUMBER_BUF: StaticCell<[u8; 8]> = StaticCell::new();
    let jedec_id = flash.blocking_jedec_id().unwrap();
    let sn = const_murmur3::murmur3_32(&jedec_id.to_le_bytes(), 0);
    let buf = SERIAL_NUMBER_BUF.init([0; 8]);
    hex::encode_to_slice(sn.to_le_bytes(), &mut buf[..]).unwrap();
    unsafe { core::str::from_utf8_unchecked(buf) }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut watchdog = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
    watchdog.set_scratch(0, 0);
    watchdog.feed();

    let usb_driver = usb::Driver::new(p.USB, Irqs);

    let usb_config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.device_release = VERSION;
        config.manufacturer = Some(MANUFACTURER);
        config.product = Some(PRODUCT);
        config.serial_number = Some(serial_number());
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config.device_class = 0xef;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        config
    };

    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 640]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();

        embassy_usb::Builder::new(usb_driver, usb_config, CONFIG_DESCRIPTOR.init([0; 640]), BOS_DESCRIPTOR.init([0; 256]), &mut [], CONTROL_BUF.init([0; 128]))
    };

    let control_class = {
        static CONTROL_STATE: StaticCell<State> = StaticCell::new();
        let state = CONTROL_STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let asic_uart_class = {
        static UART0_STATE: StaticCell<State> = StaticCell::new();
        let state = UART0_STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let asic_uart1_class = {
        static UART1_STATE: StaticCell<State> = StaticCell::new();
        let state = UART1_STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let asic_uart2_class = {
        static UART2_STATE: StaticCell<State> = StaticCell::new();
        let state = UART2_STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let asic_uart = {
        let (tx_pin, rx_pin, uart) = (p.PIN_0, p.PIN_1, p.UART0);
        static UART_TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let tx_buf = &mut UART_TX_BUF.init([0; 64])[..];
        static UART_RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let rx_buf = &mut UART_RX_BUF.init([0; 64])[..];

        embassy_rp::uart::BufferedUart::new(uart, Irqs, tx_pin, rx_pin, tx_buf, rx_buf, Default::default())
    };

    let asic_uart1 = {
        let (tx_pin, rx_pin, uart) = (p.PIN_4, p.PIN_5, p.UART1);
        static UART1_TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let tx_buf = &mut UART1_TX_BUF.init([0; 64])[..];
        static UART1_RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let rx_buf = &mut UART1_RX_BUF.init([0; 64])[..];

        embassy_rp::uart::BufferedUart::new(uart, Irqs, tx_pin, rx_pin, tx_buf, rx_buf, Default::default())
    };

    let i2c = {
        let sda = p.PIN_2;
        let scl = p.PIN_3;
        embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, Default::default())
    };

    // PSU I2C: Bit-banged on GPIO14 (SDA) / GPIO15 (SCL) at 400kHz
    let psu_i2c = control::psu::BitBangI2c::new(p.PIN_14, p.PIN_15);

    // Display I2C: I2C0 on GPIO8 (SDA) / GPIO9 (SCL) for SSD1306 OLED
    let display = {
        let sda = p.PIN_8;
        let scl = p.PIN_9;
        let display_i2c = embassy_rp::i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Default::default());
        control::display::Display::new(display_i2c)
    };

    let gpio_pins = control::gpio::Pins {
        rst0: gpio::Output::new(p.PIN_10, gpio::Level::Low),
        plug0: gpio::Input::new(p.PIN_11, gpio::Pull::None),
        rst1: gpio::Output::new(p.PIN_12, gpio::Level::Low),
        plug1: gpio::Input::new(p.PIN_13, gpio::Pull::None),
        rst2: gpio::Output::new(p.PIN_26, gpio::Level::Low),
        plug2: gpio::Input::new(p.PIN_27, gpio::Pull::None),
        psu_en: gpio::Output::new(p.PIN_25, gpio::Level::Low),
    };

    let fan_pins = {
        let mut pwm_config = pwm::Config::default();
        pwm_config.top = 1000; // 1000 steps for 0.1% resolution
        pwm_config.compare_a = 0; // Start at 0% duty cycle
        pwm_config.compare_b = 0;
        pwm_config.divider = 5.into(); // 125MHz / 5 / 1000 = 25kHz
        pwm_config.invert_a = false;
        pwm_config.phase_correct = false;
        pwm_config.enable = true; // Explicitly enable PWM
        
        let pwm = pwm::Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, pwm_config.clone());
        let tach = gpio::Input::new(p.PIN_17, gpio::Pull::Up);
        
        // Fan2: GPIO18 (PWM), GPIO19 (TACH)
        let pwm2 = pwm::Pwm::new_output_a(p.PWM_SLICE1, p.PIN_18, pwm_config.clone());
        let tach2 = gpio::Input::new(p.PIN_19, gpio::Pull::Up);
        
        // Fan3: GPIO20 (PWM), GPIO21 (TACH)
        let pwm3 = pwm::Pwm::new_output_a(p.PWM_SLICE2, p.PIN_20, pwm_config.clone());
        let tach3 = gpio::Input::new(p.PIN_21, gpio::Pull::Up);
        
        // Fan4: GPIO22 (PWM), GPIO23 (TACH)
        let pwm4 = pwm::Pwm::new_output_a(p.PWM_SLICE3, p.PIN_22, pwm_config.clone());
        let tach4 = gpio::Input::new(p.PIN_23, gpio::Pull::Up);
        
        control::fan::Pins { pwm, tach, pwm2, tach2, pwm3, tach3, pwm4, tach4 }
    };

    let pio::Pio { mut common, sm0, .. } = pio::Pio::new(p.PIO0, Irqs);
    let led = control::led::Led::new(&mut common, sm0, p.PIN_24, p.DMA_CH0.into());

    // PIO1 UART for asic_uart2 on GPIO6 (TX) / GPIO7 (RX)
    let (asic_uart2_tx, asic_uart2_rx) = {
        let pio::Pio { mut common, sm0, sm1, .. } = pio::Pio::new(p.PIO1, Irqs);
        
        let tx = pio_uart::PioUartTx::new(&mut common, sm0, p.PIN_6, 115200);
        let rx = pio_uart::PioUartRx::new(&mut common, sm1, p.PIN_7, 115200);
        (tx, rx)
    };

    unwrap!(spawner.spawn(usb_task(builder.build())));
    unwrap!(spawner.spawn(control::usb_task(control_class, psu_i2c, i2c, gpio_pins, fan_pins, led, display)));
    unwrap!(spawner.spawn(uart::usb_task(asic_uart_class, asic_uart)));
    unwrap!(spawner.spawn(uart::usb_task1(asic_uart1_class, asic_uart1)));
    unwrap!(spawner.spawn(pio_uart::pio_rx_task(asic_uart2_rx)));
    unwrap!(spawner.spawn(pio_uart::usb_task2(asic_uart2_class, asic_uart2_tx)));

    loop {
        watchdog.feed();
        Timer::after_secs(2).await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice) -> ! {
    usb.run().await
}

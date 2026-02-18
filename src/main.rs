#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::{
    adc::{self},
    bind_interrupts,
    flash::{self},
    gpio::{self},
    i2c::{self},
    peripherals::{PIO0, UART1, USB},
    pio::{self},
    usb::{self},
    Peripheral,
};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use static_cell::StaticCell;

mod control;
mod uart;

pub type AsicUart = UART1;
pub type I2cPeripheral = embassy_rp::peripherals::I2C1;
pub type I2cDriver = i2c::I2c<'static, I2cPeripheral, i2c::Async>;
pub type UsbPeripheral = embassy_rp::peripherals::USB;
pub type UsbDriver = usb::Driver<'static, UsbPeripheral>;
pub type UsbDevice = embassy_usb::UsbDevice<'static, UsbDriver>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART1_IRQ => embassy_rp::uart::BufferedInterruptHandler<UART1>;
    I2C1_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C1>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

const FLASH_SIZE: usize = 4 * 1024 * 1024;

static MANUFACTURER: &str = "256F";
static PRODUCT: &str = "EmberOne00";
/// USB bcdDevice in BCD format: 0xJJMN = version JJ.M.N
/// Major = hardware revision, minor.patch = firmware version.
/// Firmware version restarts at 0.0 with each hardware revision.
const VERSION: u16 = 0x0500;


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
        // VID:PID allocated via https://pid.codes
        let mut config = embassy_usb::Config::new(0x1209, 0x256f);
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
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        embassy_usb::Builder::new(usb_driver, usb_config, CONFIG_DESCRIPTOR.init([0; 256]), BOS_DESCRIPTOR.init([0; 256]), &mut [], CONTROL_BUF.init([0; 64]))
    };

    let control_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let asic_uart_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let asic_uart = {
        let (tx_pin, rx_pin, uart) = (p.PIN_8, p.PIN_9, p.UART1);
        static UART_TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let tx_buf = &mut UART_TX_BUF.init([0; 64])[..];
        static UART_RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let rx_buf = &mut UART_RX_BUF.init([0; 64])[..];

        embassy_rp::uart::BufferedUart::new(uart, Irqs, tx_pin, rx_pin, tx_buf, rx_buf, Default::default())
    };

    let i2c = {
        let sda = p.PIN_14;
        let scl = p.PIN_15;
        embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, Default::default())
    };

    let gpio_pins = control::gpio::Pins {
        asic_resetn: gpio::Output::new(p.PIN_11, gpio::Level::High),
        asic_pwr_en: gpio::Output::new(p.PIN_0, gpio::Level::Low),
    };

    let adc = adc::Adc::new(p.ADC, Irqs, Default::default());
    let adc_pins = control::adc::Pins {
        adc,
        vdd: adc::Channel::new_pin(p.PIN_26, gpio::Pull::None),
        vin: adc::Channel::new_pin(p.PIN_27, gpio::Pull::None),
    };

    let pio::Pio { mut common, sm0, .. } = pio::Pio::new(p.PIO0, Irqs);
    let led = control::led::Led::new(&mut common, sm0, p.PIN_1, p.DMA_CH0.into());

    unwrap!(spawner.spawn(usb_task(builder.build())));
    unwrap!(spawner.spawn(control::usb_task(control_class, i2c, gpio_pins, adc_pins, led)));
    unwrap!(spawner.spawn(uart::usb_task(asic_uart_class, asic_uart)));

    loop {
        watchdog.feed();
        Timer::after_secs(2).await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice) -> ! {
    usb.run().await
}

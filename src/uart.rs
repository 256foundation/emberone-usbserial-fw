
pub enum UartTaskError {
    Disconnected,
    UartError,
}

use super::{AsicUart, AsicUart1};
use embassy_futures::select::{select3, Either3};
use embassy_rp::{
    uart::BufferedUart,
    usb::{self},
};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender},
    driver::EndpointError,
};
use embedded_io_async::{Read, Write};

impl From<EndpointError> for UartTaskError {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => UartTaskError::Disconnected {},
        }
    }
}

impl From<embassy_rp::uart::Error> for UartTaskError {
    fn from(val: embassy_rp::uart::Error) -> Self {
        match val {
            _ => UartTaskError::UartError,
        }
    }
}

#[embassy_executor::task]
pub async fn usb_task(class: CdcAcmClass<'static, super::UsbDriver>, mut uart: BufferedUart<'static, AsicUart>) -> ! {
    let (mut tx, mut rx, mut ctrl) = class.split_with_control();

    loop {
        rx.wait_connection().await;
        let _ = pipe_uart(&mut tx, &mut rx, &mut ctrl, &mut uart).await;
    }
}

#[embassy_executor::task]
pub async fn usb_task1(class: CdcAcmClass<'static, super::UsbDriver>, mut uart: BufferedUart<'static, AsicUart1>) -> ! {
    let (mut tx, mut rx, mut ctrl) = class.split_with_control();

    loop {
        rx.wait_connection().await;
        let _ = pipe_uart1(&mut tx, &mut rx, &mut ctrl, &mut uart).await;
    }
}

/// Handle ASIC UART <-> BMC USB TTY forwarding and baudrate changes
pub async fn pipe_uart<'d, T: usb::Instance + 'd>(usb_tx: &mut Sender<'d, usb::Driver<'d, T>>, usb_rx: &mut Receiver<'d, usb::Driver<'d, T>>, ctrl: &mut ControlChanged<'d>, uart: &mut BufferedUart<'d, AsicUart>) -> Result<(), UartTaskError> {
    let mut usb_buf = [0; 64];
    let mut uart_buf = [0; 1024];

    loop {
        let (uart_tx, uart_rx) = uart.split_ref();
        let usb_read = usb_rx.read_packet(&mut usb_buf);
        let uart_read = uart_rx.read(&mut uart_buf);

        let control_change = ctrl.control_changed();

        match select3(usb_read, uart_read, control_change).await {
            // Forward data from the USB host to the UART
            Either3::First(n) => {
                let data = &usb_buf[..n?];
                uart_tx.write_all(data).await?;
            }
            // Forward data from the UART back to the USB host
            Either3::Second(n) => {
                let data = &uart_buf[..n?];
                usb_tx.write_packet(data).await?;
            }
            // Handle baudrate changes from USB CDC control requests
            Either3::Third(()) => {
                let line_coding = usb_rx.line_coding();
                let baudrate = line_coding.data_rate();
                uart.set_baudrate(baudrate);
            }
        }
    }
}

/// Handle ASIC UART1 <-> BMC USB TTY forwarding and baudrate changes
pub async fn pipe_uart1<'d, T: usb::Instance + 'd>(usb_tx: &mut Sender<'d, usb::Driver<'d, T>>, usb_rx: &mut Receiver<'d, usb::Driver<'d, T>>, ctrl: &mut ControlChanged<'d>, uart: &mut BufferedUart<'d, AsicUart1>) -> Result<(), UartTaskError> {
    let mut usb_buf = [0; 64];
    let mut uart_buf = [0; 1024];

    loop {
        let (uart_tx, uart_rx) = uart.split_ref();
        let usb_read = usb_rx.read_packet(&mut usb_buf);
        let uart_read = uart_rx.read(&mut uart_buf);

        let control_change = ctrl.control_changed();

        match select3(usb_read, uart_read, control_change).await {
            // Forward data from the USB host to the UART
            Either3::First(n) => {
                let data = &usb_buf[..n?];
                uart_tx.write_all(data).await?;
            }
            // Forward data from the UART back to the USB host
            Either3::Second(n) => {
                let data = &uart_buf[..n?];
                usb_tx.write_packet(data).await?;
            }
            // Handle baudrate changes from USB CDC control requests
            Either3::Third(()) => {
                let line_coding = usb_rx.line_coding();
                let baudrate = line_coding.data_rate();
                uart.set_baudrate(baudrate);
            }
        }
    }
}

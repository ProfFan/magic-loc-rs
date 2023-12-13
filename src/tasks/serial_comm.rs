//! Serial communication module

use core::{future::poll_fn, task::Poll};

use bbqueue::{self, GrantR};
use critical_section::RestoreState;
use embassy_executor::task;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_io_async::Write;
use esp_println::Printer;
use hal::{interrupt, peripherals::Interrupt, usb_serial_jtag::UsbSerialJtag};
use lsm6dso::ll::all_readouts::W;

static mut USB_SERIAL_READY: bool = false;
static mut USB_SERIAL_TX_BUFFER: bbqueue::BBBuffer<512> = bbqueue::BBBuffer::new();
static mut USB_SERIAL_TX_PRODUCER: Option<bbqueue::Producer<'static, 512>> = None;

static mut WAKER: AtomicWaker = AtomicWaker::new();

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut CS_RESTORE: RestoreState = RestoreState::invalid();
static mut USB_SERIAL_TX_BUFFER_FULL: bool = false;

#[defmt::global_logger]
pub struct GlobalLogger;

unsafe impl defmt::Logger for GlobalLogger {
    fn acquire() {
        unsafe {
            // safety: Must be paired with corresponding call to release(), see below
            let restore = critical_section::acquire();

            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            CS_RESTORE = restore;
        }

        unsafe { USB_SERIAL_TX_BUFFER_FULL = false };

        if unsafe { USB_SERIAL_READY } {
            // Just push the bytes to the USB serial buffer
            let grant = unsafe { USB_SERIAL_TX_PRODUCER.as_mut() }
                .unwrap()
                .grant_exact(3);

            if let Ok(mut grant) = grant {
                grant.buf()[0..2].copy_from_slice(&[0xFF, 0x00]);

                let mut size_written = 2;
                unsafe { &mut ENCODER }.start_frame(|bytes| {
                    size_written += bytes.len();
                    grant.buf()[2..].copy_from_slice(bytes)
                });

                grant.commit(size_written);
            } else {
                // Do nothing if the grant failed, we'll just drop the log

                unsafe { USB_SERIAL_TX_BUFFER_FULL = true };
            }
        } else {
            // Early boot, just print to the UART

            // Write a non-UTF8 sequence frame header that is recognized by `espflash`
            do_write(&[0xFF, 0x00]);

            unsafe { ENCODER.start_frame(do_write) }
        }
    }

    unsafe fn flush() {
        if unsafe { USB_SERIAL_READY } {
            // Do nothing
            WAKER.wake();
        } else {
            // Early boot
            Printer.flush();
        }
    }

    unsafe fn release() {
        if unsafe { USB_SERIAL_READY } {
            if unsafe { USB_SERIAL_TX_BUFFER_FULL } {
                // Do nothing
            } else {
                ENCODER.end_frame(|bytes| {
                    let grant = USB_SERIAL_TX_PRODUCER
                        .as_mut()
                        .unwrap()
                        .grant_exact(bytes.len());

                    if let Ok(mut grant) = grant {
                        grant.buf().copy_from_slice(bytes);
                        grant.commit(bytes.len());
                    } else {
                        // Do nothing if the grant failed, we'll just drop the log
                    }
                });
            }

            // Wake the serial task
            unsafe { WAKER.wake() };
        } else {
            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            ENCODER.end_frame(do_write);

            Printer.flush();
        }

        let restore = CS_RESTORE;

        critical_section::release(restore);
    }

    unsafe fn write(bytes: &[u8]) {
        if unsafe { USB_SERIAL_READY } {
            ENCODER.write(bytes, |bytes| {
                let grant = USB_SERIAL_TX_PRODUCER
                    .as_mut()
                    .unwrap()
                    .grant_exact(bytes.len());

                if let Ok(mut grant) = grant {
                    grant.buf().copy_from_slice(bytes);
                    grant.commit(bytes.len());
                } else {
                    // Do nothing if the grant failed, we'll just drop the log
                }
            });
        } else {
            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            ENCODER.write(bytes, do_write);
        }
    }
}

fn do_write(bytes: &[u8]) {
    Printer.write_bytes_assume_cs(bytes)
}

/// Write to the USB serial buffer
/// We will also need to acquire the critical section when we write to the USB serial buffer
/// to prevent interleaving with the defmt logger
pub fn write_to_usb_serial_buffer(bytes: &[u8]) -> Result<(), ()> {
    let restore = unsafe { critical_section::acquire() };

    let grant = unsafe { USB_SERIAL_TX_PRODUCER.as_mut() }
        .unwrap()
        .grant_exact(bytes.len());

    if let Ok(mut grant) = grant {
        grant.buf().copy_from_slice(bytes);
        grant.commit(bytes.len());

        unsafe { WAKER.wake() };

        unsafe { critical_section::release(restore) };

        Ok(())
    } else {
        unsafe { critical_section::release(restore) };

        Err(())
    }
}

/// The serial task
#[task]
pub async fn serial_comm_task(mut usb_serial: UsbSerialJtag<'static>) {
    defmt::info!("Serial Task Start!");

    interrupt::enable(Interrupt::USB_DEVICE, interrupt::Priority::Priority2).unwrap();

    // Initialize the BBQueue
    let (producer, mut consumer) = unsafe { USB_SERIAL_TX_BUFFER.try_split().unwrap() };
    unsafe { USB_SERIAL_TX_PRODUCER = Some(producer) };

    // TX async block
    let tx = async {
        loop {
            let tx_incoming = poll_fn(|cx| -> Poll<Result<GrantR<'_, 512>, ()>> {
                unsafe { &WAKER }.register(cx.waker());

                if let Ok(grant) = consumer.read() {
                    Poll::Ready(Ok(grant))
                } else {
                    Poll::Pending
                }
            })
            .await;

            if let Ok(grant) = tx_incoming {
                let bytes = grant.buf();

                let len = bytes.len();
                usb_serial.write_all(bytes).await.unwrap();

                grant.release(len);
            }
        }
    };

    unsafe { USB_SERIAL_READY = true };

    tx.await;
}

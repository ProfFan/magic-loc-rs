#![deny(unsafe_op_in_unsafe_fn)]

//! Serial communication module

use core::{future::poll_fn, sync::atomic::AtomicBool, task::Poll};

use bbqueue::{self, GrantR};
use critical_section::RestoreState;
use embassy_executor::task;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_io_async::Write;
use esp_println::Printer;
use hal::{interrupt, peripherals::Interrupt, usb_serial_jtag::UsbSerialJtag};

use hal::macros::ram;

static mut USB_SERIAL_READY: AtomicBool = AtomicBool::new(false);
static mut USB_SERIAL_TX_BUFFER: bbqueue::BBBuffer<2048> = bbqueue::BBBuffer::new();
static mut USB_SERIAL_TX_PRODUCER: Option<bbqueue::Producer<'static, 2048>> = None;

static mut WAKER: AtomicWaker = AtomicWaker::new();

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut CS_RESTORE: RestoreState = RestoreState::invalid();
static mut TAKEN: bool = false;
static mut LOGGER_BUFFER: [u8; 2048] = [0; 2048];
static mut BUFFER_CURSOR: usize = 0;

#[defmt::global_logger]
pub struct GlobalLogger;

unsafe impl defmt::Logger for GlobalLogger {
    #[ram]
    fn acquire() {
        unsafe {
            // safety: Must be paired with corresponding call to release(), see below
            let restore = critical_section::acquire();

            // Compiler fence to prevent reordering of the above critical section with the
            // subsequent access to the `TAKEN` flag.
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            if TAKEN {
                panic!("defmt logger already taken!");
            }

            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            CS_RESTORE = restore;

            TAKEN = true;
        }

        if unsafe { USB_SERIAL_READY.load(core::sync::atomic::Ordering::Relaxed) } {
            unsafe { LOGGER_BUFFER[0..2].copy_from_slice(&[0xFF, 0x00]) };

            let mut size_written = 2;
            unsafe { &mut ENCODER }.start_frame(|bytes| {
                if size_written + bytes.len() > unsafe { LOGGER_BUFFER }.len() {
                    // Buffer overflow
                    return;
                }
                unsafe { &mut LOGGER_BUFFER[size_written..size_written + bytes.len()] }
                    .copy_from_slice(bytes);
                size_written += bytes.len();
            });

            unsafe { BUFFER_CURSOR = size_written };
        } else {
            // Early boot, just print to the UART

            // Write a non-UTF8 sequence frame header that is recognized by `espflash`
            do_write(&[0xFF, 0x00]);

            unsafe { ENCODER.start_frame(do_write) }
        }
    }

    #[ram]
    unsafe fn flush() {
        if unsafe { USB_SERIAL_READY.load(core::sync::atomic::Ordering::Relaxed) } {
            // Do nothing
            unsafe {
                WAKER.wake();
            }
        } else {
            // Early boot
            Printer.flush();
        }
    }

    #[ram]
    unsafe fn release() {
        if unsafe { USB_SERIAL_READY.load(core::sync::atomic::Ordering::Relaxed) } {
            unsafe { &mut ENCODER }.end_frame(|bytes| {
                let cursor = unsafe { BUFFER_CURSOR };

                if cursor + bytes.len() > unsafe { LOGGER_BUFFER }.len() {
                    // Buffer overflow
                    return;
                }

                unsafe { &mut LOGGER_BUFFER[cursor..cursor + bytes.len()] }.copy_from_slice(bytes);
                unsafe { BUFFER_CURSOR += bytes.len() };
            });

            // Write to actual producer
            let cursor = unsafe { BUFFER_CURSOR };
            let bytes = &unsafe { LOGGER_BUFFER }[0..cursor];

            let _ = write_to_usb_serial_buffer(bytes);

            // Wake the serial task
            unsafe { WAKER.wake() };
        } else {
            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            unsafe { &mut ENCODER }.end_frame(do_write);

            Printer.flush();
        }

        unsafe {
            TAKEN = false;

            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            critical_section::release(CS_RESTORE);
        }
    }

    #[ram]
    unsafe fn write(bytes: &[u8]) {
        if unsafe { USB_SERIAL_READY.load(core::sync::atomic::Ordering::Relaxed) } {
            unsafe { &mut ENCODER }.write(bytes, |bytes| {
                let cursor = unsafe { BUFFER_CURSOR };

                if cursor + bytes.len() > unsafe { LOGGER_BUFFER }.len() {
                    // Buffer overflow
                    return;
                }

                unsafe { &mut LOGGER_BUFFER[cursor..cursor + bytes.len()] }.copy_from_slice(bytes);
                unsafe { BUFFER_CURSOR += bytes.len() };
            });
        } else {
            // safety: accessing the `static mut` is OK because we have acquired a critical
            // section.
            unsafe { &mut ENCODER }.write(bytes, do_write);
        }
    }
}

#[inline]
fn do_write(bytes: &[u8]) {
    Printer.write_bytes_assume_cs(bytes)
}

/// Write to the USB serial buffer
/// We will also need to acquire the critical section when we write to the USB serial buffer
/// to prevent interleaving with the defmt logger
#[ram]
pub fn write_to_usb_serial_buffer(bytes: &[u8]) -> Result<(), ()> {
    let restore = unsafe { critical_section::acquire() };
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    let grant = unsafe { USB_SERIAL_TX_PRODUCER.as_mut() }
        .unwrap()
        .grant_exact(bytes.len());

    if let Ok(mut grant) = grant {
        grant.buf().copy_from_slice(bytes);
        grant.commit(bytes.len());

        unsafe { WAKER.wake() };

        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { critical_section::release(restore) };

        Ok(())
    } else {
        unsafe { WAKER.wake() };

        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { critical_section::release(restore) };

        Err(())
    }
}

/// The serial task
#[task]
#[ram]
pub async fn serial_comm_task(mut usb_serial: UsbSerialJtag<'static>) {
    defmt::info!("Serial Task Start!");

    interrupt::enable(Interrupt::USB_DEVICE, interrupt::Priority::Priority1).unwrap();

    // Initialize the BBQueue
    let (producer, mut consumer) = unsafe { USB_SERIAL_TX_BUFFER.try_split().unwrap() };
    unsafe { USB_SERIAL_TX_PRODUCER = Some(producer) };

    // TX async block
    let tx = async {
        loop {
            let tx_incoming = poll_fn(|cx| -> Poll<Result<GrantR<'_, 2048>, ()>> {
                if let Ok(grant) = consumer.read() {
                    Poll::Ready(Ok(grant))
                } else {
                    unsafe { &WAKER }.register(cx.waker());

                    Poll::Pending
                }
            })
            .await;

            if let Ok(grant) = tx_incoming {
                let bytes = grant.buf();

                let len = bytes.len().clamp(0, 64);
                let written = usb_serial.write(&bytes[..len]).await.unwrap();

                grant.release(written);
            }

            embassy_futures::yield_now().await;
        }
    };

    unsafe { USB_SERIAL_READY.store(true, core::sync::atomic::Ordering::Relaxed) };

    tx.await;
}

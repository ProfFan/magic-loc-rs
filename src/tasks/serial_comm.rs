// Serial communication module

use core::sync::atomic::AtomicBool;

use critical_section::RestoreState;
use embassy_executor::task;
use esp_println::Printer;
use hal::usb_serial_jtag::UsbSerialJtag;

static mut USB_SERIAL: Option<UsbSerialJtag> = None;
static mut USB_SERIAL_TX_BUFFER: [u8; 1024] = [0; 1024];

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut CS_RESTORE: RestoreState = RestoreState::invalid();

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

        if let Some(usb_serial) = unsafe { USB_SERIAL.as_mut() } {
            usb_serial.write(&[0xFF, 0x00]).ok();
        }
        // Write a non-UTF8 sequence frame header that is recognized by `espflash`
        do_write(&[0xFF, 0x00]);

        unsafe { ENCODER.start_frame(do_write) }
    }

    unsafe fn release() {
        // safety: accessing the `static mut` is OK because we have acquired a critical
        // section.
        ENCODER.end_frame(do_write);

        Printer.flush();

        let restore = CS_RESTORE;

        critical_section::release(restore);
    }

    unsafe fn flush() {
        Printer.flush();
    }

    unsafe fn write(bytes: &[u8]) {
        // safety: accessing the `static mut` is OK because we have acquired a critical
        // section.
        ENCODER.write(bytes, do_write);
    }
}

fn do_write(bytes: &[u8]) {
    Printer.write_bytes_assume_cs(bytes)
}

// The serial task

#[task]
pub async fn serial_comm_task(mut usb_serial: UsbSerialJtag<'static>) {
    // Safe because this is an exclusive write to the static variable
    unsafe {
        USB_SERIAL = Some(usb_serial);
    }

    loop {
    }
}

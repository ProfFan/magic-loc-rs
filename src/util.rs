use core::future::Future;

use embassy_sync::blocking_mutex::raw::RawMutex;
/// Wait on a function that returns `nb::Result` asynchronously.
///
/// When `f` returns `nb::Error::WouldBlock`, this function will wait for
/// the GPIO output to go high, and then call `f` again.
// #[inline]
// pub async fn nonblocking_wait_int<T, E>(
//     mut f: impl FnMut() -> nb::Result<T, E>,
//     int_gpio: &mut impl embedded_hal_async::digital::Wait,
// ) -> Result<T, E> {
//     loop {
//         let v = f();
//         match v {
//             Ok(t) => return Ok(t),
//             Err(nb::Error::Other(e)) => return Err(e),
//             Err(nb::Error::WouldBlock) => {
//                 int_gpio.wait_for_high().await.unwrap();
//                 continue;
//             }
//         }
//     }
// }
use hal::prelude::nb;

/// Wait on a function that returns `nb::Result` asynchronously that can be cancelled.
///
/// When `f` returns `nb::Error::WouldBlock`, this function will wait for
/// the GPIO output to go high, and then call `f` again.
#[inline]
pub async fn nonblocking_wait_cancellable<T, E, MUTEX>(
    mut f: impl FnMut() -> Result<T, nb::Error<E>>,
    int_gpio: &mut impl embedded_hal_async::digital::Wait,
    cancel: &embassy_sync::signal::Signal<MUTEX, bool>,
) -> Result<Result<T, E>, ()>
where
    MUTEX: RawMutex,
{
    loop {
        let v = f();

        match v {
            Ok(t) => return Ok(Ok(t)),
            Err(nb::Error::Other(e)) => return Ok(Err(e)),
            Err(nb::Error::WouldBlock) => {
                let result =
                    embassy_futures::select::select(int_gpio.wait_for_high(), cancel.wait()).await;

                if let embassy_futures::select::Either::Second(_) = result {
                    return Err(());
                }

                continue;
            }
        }
    }
}

/// Wait on a function that returns `nb::Result` asynchronously that can be cancelled.
///
/// When `f` returns `nb::Error::WouldBlock`, this function will wait for
/// the GPIO output to go high, and then call `f` again.
#[inline]
pub async fn nonblocking_wait<T, E>(
    mut f: impl FnMut() -> Result<T, nb::Error<E>>,
    int_gpio: &mut impl embedded_hal_async::digital::Wait,
) -> Result<T, E> {
    loop {
        let v = f();

        match v {
            Ok(t) => return Ok(t),
            Err(nb::Error::Other(e)) => return Err(e),
            Err(nb::Error::WouldBlock) => {
                int_gpio.wait_for_high().await.unwrap();
                continue;
            }
        }
    }
}

/// Wait on a function that returns `nb::Result` asynchronously.
///
/// NOTE: This is a more advanced version of `nonblocking_wait` that allows
/// the function to take a mutable reference to some argument.
///
/// When `f` returns `nb::Error::WouldBlock`, this function will wait for
/// the GPIO output to go high, and then call `f` again.
#[inline]
#[allow(dead_code)]
pub async fn nonblocking_wait_int<'a, T: 'a, E, A>(
    mut f: impl FnMut(&'a mut A) -> Result<T, (nb::Error<E>, &'a mut A)>,
    int_gpio: &mut impl embedded_hal_async::digital::Wait,
    mut arg: &'a mut A,
) -> Result<T, E> {
    loop {
        let v = f(arg);

        match v {
            Ok(t) => return Ok(t),
            Err((nb::Error::Other(e), _)) => return Err(e),
            Err((nb::Error::WouldBlock, arg_)) => {
                int_gpio.wait_for_high().await.unwrap();
                arg = arg_;
                continue;
            }
        }
    }
}

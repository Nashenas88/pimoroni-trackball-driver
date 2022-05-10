# pimoroni-trackball-driver

A simple driver for the [Pimoroni Trackball](https://shop.pimoroni.com/products/trackball-breakout?variant=27672765038675) written with [embedded-hal](https://docs.rs/embedded-hal).

The imlpementation is based off the implementations written in Python (https://github.com/pimoroni/trackball-python) and C++ (https://github.com/pimoroni/pimoroni-pico/blob/cb958a7c8a73cdc51873058be4918b893d2c7797/drivers/trackball/trackball.cpp).

## Example

```rust
#![no_std]
#![no_main]

use bsp::hal::gpio::bank0::{Gpio20, Gpio21, Gpio22};
use bsp::hal::gpio::{FunctionI2C, Interrupt, Pin, PullUpInput};
use bsp::pac::I2C0;
use core::cell::RefCell;
use core::iter::once;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_time::duration::Extensions as _;
use embedded_time::rate::Extensions as _;
use panic_probe as _;
use pimoroni_trackball as trackball;
use rp_pico as bsp;
use bsp::hal::clocks::{init_clocks_and_plls, Clock};
use bsp::hal::pac::{self, interrupt};
use bsp::hal::sio::Sio;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::Timer;
use bsp::hal::{self, I2C};
use bsp::XOSC_CRYSTAL_FREQ;
use embedded_hal::timer::CountDown;
use smart_leds::{brightness, RGB8};
use trackball::{Trackball, TrackballBuilder};

const LED_ANIM_TIME_US: u32 = 32_000; // 60Hz
type TrackballWithPins = Trackball<
    I2C<I2C0, (Pin<Gpio20, FunctionI2C>, Pin<Gpio21, FunctionI2C>)>,
    Pin<Gpio22, PullUpInput>,
>;
static TRACKBALL: Mutex<RefCell<Option<TrackballWithPins>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    // Configure the i2c peripheral for the trackball.
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        pins.gpio20.into_mode::<hal::gpio::FunctionI2C>(),
        pins.gpio21.into_mode::<hal::gpio::FunctionI2C>(),
        100.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    // Construct the I2C interface from the peripheral.
    let interface = trackball::I2CInterface::new(i2c);

    // Construct the trackball with the I2C interface and an interrupt pin.
    let mut trackball = TrackballBuilder::new(interface)
        .interrupt_pin(pins.gpio22.into_pull_up_input())
        .build();

    // Initialize the trackball, setting the interrupt enabled when the input pin triggers low.
    let _ = trackball
        .init(|interrupt_pin| interrupt_pin.set_interrupt_enabled(Interrupt::EdgeLow, true));

    trackball.set_rgbw(0, 0, 0, 0).unwrap();
    let mut n: u8 = 128;

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        // Also ensure to assign the trackball data for the interrupt before the interrupt handler
        // is enabled.
        cortex_m::interrupt::free(|cs| *TRACKBALL.borrow(cs).borrow_mut() = Some(trackball));
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // Update the LED on the trackball to display a color wheel.
    loop {
        delay.start(LED_ANIM_TIME_US.microseconds());
        let _ = nb::block!(delay.wait());

        let rgb = brightness(once(wheel(n)), 32).next().unwrap();
        n = n.wrapping_add(1);
        cortex_m::interrupt::free(|cs| {
            let mut trackball = TRACKBALL.borrow(cs).borrow_mut();
            let trackball = trackball.as_mut().unwrap();
            trackball.set_rgbw(rgb.r, rgb.g, rgb.b, 0).unwrap();
        });
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    let data = cortex_m::interrupt::free(|cs| {
        let mut trackball = TRACKBALL.borrow(cs).borrow_mut();
        let trackball = trackball.as_mut().unwrap();
        // Read the trackball data on interrupt.
        let data = trackball.read().unwrap();
        // Ensure the interrupt is cleared before we continue.
        trackball
            .interrupt()
            .unwrap()
            .clear_interrupt(Interrupt::EdgeLow);
        data
    });

    // Log the state of the trackball data for the current state.
    if data.up > 0 || data.down > 0 || data.left > 0 || data.right > 0 || data.switch_changed {
        info!(
            "{} {} {} {} {} {}",
            data.up,
            data.down,
            data.left,
            data.right,
            if data.switch_changed { 1 } else { 0 },
            if data.switch_pressed { 1 } else { 0 }
        );
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}
```

### License
[MIT](./LICENSE)

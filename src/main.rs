//! # DHT11 Example
//!
//! This application demonstrates how to read a DHT11 sensor on the RP2040.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//! In this example, the DHT11 data pin should be connected to GPIO28.
//!
//! NOTE: The DHT11 driver only works reliably when compiled in release mode.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use dht_sensor::*;
use panic_halt as _;

// Alias for our HAL crate
use bsp::entry;
use rp2040_hal as hal;
use rp_pico as bsp;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

use defmt::*;
use defmt_rtt as _;

// Some traits we need
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use hal::gpio::dynpin::DynPin;
use hal::Clock;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// A wrapper for DynPin, implementing both InputPin and OutputPin, to simulate
/// an open-drain pin as needed by the wire protocol the DHT11 sensor speaks.
/// https://how2electronics.com/interfacing-dht11-temperature-humidity-sensor-with-raspberry-pi-pico/
struct InOutPin {
    inner: DynPin,
}

impl InOutPin {
    fn new(inner: DynPin) -> Self {
        Self { inner }
    }
}

impl InputPin for InOutPin {
    type Error = rp2040_hal::gpio::Error;
    fn is_high(&self) -> Result<bool, <Self as embedded_hal::digital::v2::InputPin>::Error> {
        self.inner.is_high()
    }
    fn is_low(&self) -> Result<bool, <Self as embedded_hal::digital::v2::InputPin>::Error> {
        self.inner.is_low()
    }
}

impl OutputPin for InOutPin {
    type Error = rp2040_hal::gpio::Error;
    fn set_low(&mut self) -> Result<(), <Self as embedded_hal::digital::v2::OutputPin>::Error> {
        // To actively pull the pin low, it must also be configured as a (readable) output pin
        self.inner.into_readable_output();
        // In theory, we should set the pin to low first, to make sure we never actively
        // pull it up. But if we try it on the input pin, we get Err(Gpio(InvalidPinType)).
        self.inner.set_low()?;
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), <Self as embedded_hal::digital::v2::OutputPin>::Error> {
        // To set the open-drain pin to high, just disable the output driver by changing the
        // pin to input mode with pull-up. That way, the DHT11 can still pull the data line down
        // to send its response.
        self.inner.into_pull_up_input();
        Ok(())
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, assigns GPIO 28 to the
/// DHT11 driver, and takes a single measurement.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // let mut io_pin = InOutPin::new(pins.gpio15.into());
    let mut sound_pin = pins.gpio15.into_pull_down_input();
    //    let mut dh11 = Dht11::new(io_pin);
    //    delay.delay_ms(4000);
    //    match dht11.perform_measurement(&mut delay) {
    //        Ok(_) => {
    //            info!("ok");
    //        }
    //        Err(err) => {
    //            info!("err");
    //        }
    //    }
    let mut led_pin = pins.gpio14.into_push_pull_output();

    loop {
        // cortex_m::asm::wfi();
        if sound_pin.is_high().unwrap() {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
        // match dht11::Reading::read(&mut delay, &mut io_pin) {
        //     Ok(_) => info!("ok"),
        //     Err(_) => info!("err"),
        // }
    }
}

// End of file

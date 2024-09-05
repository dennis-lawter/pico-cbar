#![no_std]
#![no_main]

use rp_pico as bsp;

use bsp::entry;
//use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::clocks::Clock;
use bsp::hal::clocks::ClocksManager;
use bsp::hal::pac;
use bsp::hal::sio::Sio;
use bsp::hal::watchdog::Watchdog;
use bsp::Pins;
use cortex_m::delay::Delay;

#[allow(dead_code)]
struct Pico {
    //    pub pac: pac::Peripherals,
    //    pub core: pac::CorePeripherals,
    pub watchdog: Watchdog,
    //    pub sio: Sio,
    pub clocks: ClocksManager,
    pub delay: Delay,
    pub pins: Pins,
}
impl Default for Pico {
    fn default() -> Self {
        let mut pac = pac::Peripherals::take().unwrap();
        let core = pac::CorePeripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let sio = Sio::new(pac.SIO);

        let external_xtal_freq_hz = 12_000_000u32;
        let clocks = init_clocks_and_plls(
            external_xtal_freq_hz,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        let pins = bsp::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        Self {
            //          pac,
            //          core,
            watchdog,
            //            sio,
            clocks,
            delay,
            pins,
        }
    }
}

#[entry]
fn main() -> ! {
    let mut pico = Pico::default();

    let mut led_pin = pico.pins.led.into_push_pull_output();

    loop {
        led_pin.set_high().unwrap();
        pico.delay.delay_ms(500);

        led_pin.set_low().unwrap();
        pico.delay.delay_ms(500);
    }
}

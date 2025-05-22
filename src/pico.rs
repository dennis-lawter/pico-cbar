use rp_pico as bsp;

use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::clocks::Clock;
use bsp::hal::clocks::ClocksManager;
use bsp::hal::pac;
use bsp::hal::sio::Sio;
use bsp::hal::watchdog::Watchdog;
use cortex_m::delay::Delay;
use cortex_m::prelude::_embedded_hal_PwmPin;
use embedded_hal::digital::InputPin;

type Key0Pin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio15, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;
type Key1Pin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;
type Key2Pin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;

type BuzzerPwmSlice = hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>;
type BuzzerPinChannel = hal::pwm::Channel<BuzzerPwmSlice, hal::pwm::A>;

pub enum KeyNames {
    Tilt,
    MetalHit,
    BodyHit,
    // Whoosh,
}

#[allow(dead_code)]
pub struct Pico {
    pub watchdog: Watchdog,
    pub clocks: ClocksManager,
    pub delay: Delay,
    pub buzzer_channel_ptr: *mut BuzzerPinChannel,
    pub buzzer_pwm_slice_ptr: *mut BuzzerPwmSlice,
    pub key0: Key0Pin,
    pub key1: Key1Pin,
    pub key2: Key2Pin,
}
impl Pico {
    pub fn set_amplitude(&mut self, amplitude: u8) {
        //let scaled_amplitude = (amplitude as u32 * (5_535 + 1)) / 256;
        let scaled_amplitude = amplitude as u16;
        unsafe {
            (*self.buzzer_channel_ptr).set_duty(scaled_amplitude as u16);
        }
    }

    pub fn is_key_pressed(&mut self, key: KeyNames) -> bool {
        match key {
            KeyNames::Tilt => self.key0.is_low().unwrap(),
            KeyNames::MetalHit => self.key1.is_low().unwrap(),
            KeyNames::BodyHit => self.key2.is_low().unwrap(),
            // KeyNames::Whoosh => self.key3.is_low().unwrap(),
        }
    }
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

        let key0 = pins.gpio15.into_pull_up_input();
        let key1 = pins.gpio0.into_pull_up_input();
        let key2 = pins.gpio1.into_pull_up_input();

        let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

        unsafe {
            // Configure buzzer PWM slice
            let buzzer_pwm_slice_ptr: *mut BuzzerPwmSlice =
                &mut pwm_slices.pwm2 as *mut BuzzerPwmSlice;

            let buzzer_channel_ptr =
                &mut (*buzzer_pwm_slice_ptr).channel_a as *mut BuzzerPinChannel;
            (*buzzer_channel_ptr).output_to(pins.gpio4);

            //let top: u64 = 65_535;
            //let system_clock_freq: u64 = clocks.system_clock.freq().to_Hz() as u64;
            //let target_pwm_freq: u64 = 22_050 * 256; // Match PWM frequency with sample rate

            //let div_comb = (system_clock_freq * 16) / (target_pwm_freq * (top + 1));
            //let div_int = (div_comb / 16) as u8;
            //let div_frac = (div_comb % 16) as u8;

            let top = 512;
            let div_int = 1;
            let div_frac = 0;
            (*buzzer_pwm_slice_ptr).set_ph_correct();
            (*buzzer_pwm_slice_ptr).set_div_int(div_int);
            (*buzzer_pwm_slice_ptr).set_div_frac(div_frac);
            (*buzzer_pwm_slice_ptr).set_top(top as u16);
            (*buzzer_pwm_slice_ptr).enable();

            Self {
                watchdog,
                clocks,
                delay,
                buzzer_channel_ptr,
                buzzer_pwm_slice_ptr,
                key0,
                key1,
                key2,
            }
        }
    }
}

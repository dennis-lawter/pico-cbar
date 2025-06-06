use embedded_hal::digital::OutputPin;
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

use crate::button::ButtonName;
use crate::wav::Wav;

type TiltTriggerButtonPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio15, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;
type TiltResetButtonPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio14, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;
type BlackButtonPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;
type RedButtonPin =
    hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionSioInput, hal::gpio::PullUp>;

type Led1Pin = hal::gpio::Pin<
    hal::gpio::bank0::Gpio18,
    hal::gpio::FunctionSio<hal::gpio::SioOutput>,
    hal::gpio::PullDown,
>;
type Led2Pin = hal::gpio::Pin<
    hal::gpio::bank0::Gpio19,
    hal::gpio::FunctionSio<hal::gpio::SioOutput>,
    hal::gpio::PullDown,
>;

type BuzzerPwmSlice = hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>;
type BuzzerPinChannel = hal::pwm::Channel<BuzzerPwmSlice, hal::pwm::A>;

pub enum LedNames {
    Led1,
    Led2,
}

#[allow(dead_code)]
pub struct Pico {
    pub watchdog: Watchdog,
    pub clocks: ClocksManager,
    pub delay: Delay,
    pub buzzer_channel_ptr: *mut BuzzerPinChannel,
    pub buzzer_pwm_slice_ptr: *mut BuzzerPwmSlice,
    pub tilt_trigger_button: TiltTriggerButtonPin,
    pub tilt_reset_button: TiltResetButtonPin,
    pub black_button: BlackButtonPin,
    pub red_button: RedButtonPin,
    pub led1: Led1Pin,
    pub led2: Led2Pin,
}
impl Pico {
    pub fn set_amplitude(&mut self, amplitude: u8) {
        //let scaled_amplitude = (amplitude as u32 * (5_535 + 1)) / 256;
        let scaled_amplitude = amplitude as u16;
        unsafe {
            (*self.buzzer_channel_ptr).set_duty(scaled_amplitude as u16);
        }
    }

    pub fn set_led_state(&mut self, led: LedNames, state: bool) {
        match (led, state) {
            (LedNames::Led1, true) => self.led1.set_high().unwrap(),
            (LedNames::Led1, false) => self.led1.set_low().unwrap(),
            (LedNames::Led2, true) => self.led2.set_high().unwrap(),
            (LedNames::Led2, false) => self.led2.set_low().unwrap(),
        }
    }

    pub fn is_button_pressed(&mut self, key: ButtonName) -> bool {
        match key {
            ButtonName::TiltTrigger => self.tilt_trigger_button.is_low().unwrap(),
            ButtonName::TiltReset => self.tilt_reset_button.is_low().unwrap(),
            ButtonName::MetalHit => self.black_button.is_low().unwrap(),
            ButtonName::BodyHit => self.red_button.is_low().unwrap(),
        }
    }

    pub fn play_wav_blocking(&mut self, wav: &Wav) {
        let sample_rate = wav.sample_rate;
        let cpu_delay_between_samples_in_us = 1_000_000 / sample_rate;
        let sound = wav.data_ref;
        let data_start = 0x2C;
        let mut data_end = data_start + wav.chunk_len;
        if data_end > sound.len() {
            data_end = sound.len();
        }
        for i in data_start..data_end {
            self.set_amplitude(sound[i]);

            self.delay.delay_us(cpu_delay_between_samples_in_us);
        }
        self.set_amplitude(0);
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

        let tilt_trigger_button = pins.gpio15.into_pull_up_input();
        let tilt_reset_button = pins.gpio14.into_pull_up_input();
        let black_button = pins.gpio0.into_pull_up_input();
        let red_button = pins.gpio1.into_pull_up_input();

        let led1 = pins.gpio18.into_push_pull_output();
        let led2 = pins.gpio19.into_push_pull_output();
        let mut pwr_led = pins.gpio13.into_push_pull_output();
        pwr_led.set_high().unwrap();

        let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

        unsafe {
            // Configure buzzer PWM slice
            let buzzer_pwm_slice_ptr: *mut BuzzerPwmSlice =
                &mut pwm_slices.pwm2 as *mut BuzzerPwmSlice;

            let buzzer_channel_ptr =
                &mut (*buzzer_pwm_slice_ptr).channel_a as *mut BuzzerPinChannel;
            (*buzzer_channel_ptr).output_to(pins.gpio4);

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
                tilt_trigger_button,
                tilt_reset_button,
                black_button,
                red_button,
                led1,
                led2,
            }
        }
    }
}

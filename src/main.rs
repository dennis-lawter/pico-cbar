#![no_std]
#![no_main]

use rp_pico as bsp;

use bsp::entry;
//use defmt::*;
use defmt_rtt as _;
//use embedded_hal::digital::OutputPin;
use panic_probe as _;

use bsp::hal;
use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::clocks::Clock;
use bsp::hal::clocks::ClocksManager;
use bsp::hal::pac;
use bsp::hal::sio::Sio;
use bsp::hal::watchdog::Watchdog;
//use bsp::Pins;
use cortex_m::delay::Delay;
use cortex_m::prelude::_embedded_hal_PwmPin;
//type PwmSlice = bsp::hal::pwm::Slice<bsp::hal::pwm::Pwm6, bsp::hal::pwm::FreeRunning>;
//type PwmChannel = bsp::hal::pwm::Channel<bsp::hal::pwm::Pwm6, bsp::hal::pwm::FreeRunning, bsp::hal::pwm::B>;
//type PwmChannel = bsp::hal::pwm::Channel<bsp::hal::pwm::Slice<bsp::hal::pwm::Pwm6, bsp::hal::pwm::FreeRunning>, bsp::hal::pwm::A>;
//type PwmSlice = bsp::hal::pwm::Slice<bsp::hal::pwm::Pwm6, bsp::hal::pwm::FreeRunning>;
type BuzzerPwmSlice = hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>;
type BuzzerPinChannel = hal::pwm::Channel<BuzzerPwmSlice, hal::pwm::A>;

#[allow(dead_code)]
struct Pico {
    //    pub pac: pac::Peripherals,
    //    pub core: pac::CorePeripherals,
    pub watchdog: Watchdog,
    //    pub sio: Sio,
    pub clocks: ClocksManager,
    pub delay: Delay,
    //pub pins: Pins,
    //pub pwm6: PwmSlice,
    //pub pwm_channel: PwmChannel,
    pub buzzer_channel_ptr: *mut BuzzerPinChannel,
    pub buzzer_pwm_slice_ptr: *mut BuzzerPwmSlice,
}
impl Pico {
    fn set_amplitude(&mut self, amplitude: u16) {
        unsafe {
            (*self.buzzer_channel_ptr).set_duty(amplitude);
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

        let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

        /*
                let pwm6 = &mut pwm_slices.pwm6;
                pwm6.set_ph_correct();
                pwm6.set_top(65_535);
                pwm6.set_div_int(1);
                pwm6.set_div_frac(0);
                pwm6.enable();
        */

        unsafe {
            // Configure buzzer PWM slice
            let buzzer_pwm_slice_ptr: *mut BuzzerPwmSlice =
                &mut pwm_slices.pwm2 as *mut BuzzerPwmSlice;

            let buzzer_channel_ptr =
                &mut (*buzzer_pwm_slice_ptr).channel_a as *mut BuzzerPinChannel;
            (*buzzer_channel_ptr).output_to(pins.gpio4);
            //(*buzzer_channel_ptr).set_duty(0);

            (*buzzer_pwm_slice_ptr).set_ph_correct();
            (*buzzer_pwm_slice_ptr).set_div_int(0);
            (*buzzer_pwm_slice_ptr).set_div_frac(0);
            (*buzzer_pwm_slice_ptr).set_top(65_535);
            (*buzzer_pwm_slice_ptr).enable();

            Self {
                //          pac,
                //          core,
                watchdog,
                //            sio,
                clocks,
                delay,
                //pins,
                //pwm6: (*pwm6),
                //pwm_channel,
                buzzer_channel_ptr,
                buzzer_pwm_slice_ptr,
            }
        }
    }
}

const CBAR_MISS1: &[u8; 7710] = include_bytes!("../sfx/cbar_miss1.wav");
//const CBAR_MISS1_BR: usize = 16;
const CBAR_MISS1_SR: usize = 22_050;

#[entry]
fn main() -> ! {
    let mut pico = Pico::default();

    loop {
        play_crowbar_miss(&mut pico);
        pico.set_amplitude(0);
        pico.delay.delay_ms(1_000);
    }
}

fn play_crowbar_miss(pico: &mut Pico) {
    // Extract 16-bit audio samples from the WAV file
    let audio_data = &CBAR_MISS1[44..]; // Skip the WAV header
    let mut idx = 0;

    while idx < audio_data.len() {
        let sample = (audio_data[idx] as u16) | ((audio_data[idx + 1] as u16) << 8);
        //let amplitude = sample.wrapping_add(32_767); // Convert signed 16-bit to unsigned 16-bit
        pico.set_amplitude(sample);

        // Wait to maintain sample rate
        pico.delay.delay_us(1_000_000 / CBAR_MISS1_SR as u32);
        idx += 2;
    }
}

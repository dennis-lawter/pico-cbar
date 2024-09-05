#![no_std]
#![no_main]

use rp_pico as bsp;

use bsp::entry;
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
type BuzzerPwmSlice = hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>;
type BuzzerPinChannel = hal::pwm::Channel<BuzzerPwmSlice, hal::pwm::A>;

#[allow(dead_code)]
struct Pico {
    pub watchdog: Watchdog,
    pub clocks: ClocksManager,
    pub delay: Delay,
    pub buzzer_channel_ptr: *mut BuzzerPinChannel,
    pub buzzer_pwm_slice_ptr: *mut BuzzerPwmSlice,
}
impl Pico {
    fn set_amplitude(&mut self, amplitude: u8) {
        //let scaled_amplitude = (amplitude as u32 * (5_535 + 1)) / 256;
        let scaled_amplitude = amplitude as u16 * 2;
        unsafe {
            (*self.buzzer_channel_ptr).set_duty(scaled_amplitude as u16);
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
            }
        }
    }
}

const CBAR_MISS1: &[u8; 7710] = include_bytes!("../sfx/cbar_miss1.wav");
const CBAR_HITBOD1: &[u8; 7048] = include_bytes!("../sfx/cbar_hitbod1.wav");

#[entry]
fn main() -> ! {
    let mut pico = Pico::default();

    loop {
        play_8b_sound(&mut pico, CBAR_MISS1);
        pico.set_amplitude(0);
        pico.delay.delay_ms(1_000);

        play_8b_sound(&mut pico, CBAR_HITBOD1);
        pico.set_amplitude(0);
        pico.delay.delay_ms(1_000);
    }
}

const SAMPLE_RATE: u32 = 22_050;
const SAMPLE_WAIT: u32 = 1_000_000 / SAMPLE_RATE; // Microseconds per sample

fn play_8b_sound(pico: &mut Pico, sound: &[u8]) {
    for i in 44..sound.len() {
        pico.set_amplitude(sound[i]);

        // Wait to maintain sample rate
        pico.delay.delay_us(SAMPLE_WAIT);
    }
}

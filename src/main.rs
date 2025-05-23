#![no_std]
#![no_main]

mod library;
mod pico;
mod wav;

use library::Library;
use pico::KeyNames;
use pico::Pico;

use rp_pico as bsp;

use bsp::entry;

#[entry]
fn main() -> ! {
    let mut pico = Pico::default();
    let snd_lib = Library::default();

    let mut tilt_trigger_time_pressed_in_us = 0usize;
    let mut tilt_trigger_time_released_in_us = 0usize;
    let tilt_trigger_time_limit_in_us = 10_000usize;
    let mut tilt_trigger_logically_pressed = false;

    let mut tilt_reset_time_pressed_in_us = 0usize;
    let mut tilt_reset_time_released_in_us = 0usize;
    let tilt_reset_time_limit_in_us = 10_000usize;
    let mut tilt_reset_logically_pressed = false;

    let mut needs_to_be_reset = false;

    let mut loop_count = 0usize;

    loop {
        loop_count = loop_count.wrapping_add(1);
        let tilt_trigger_pressed = pico.is_key_pressed(KeyNames::TiltTrigger);
        let tilt_reset_pressed = pico.is_key_pressed(KeyNames::TiltReset);

        if tilt_trigger_pressed {
            tilt_trigger_time_pressed_in_us += 1;
            tilt_trigger_time_released_in_us = 0;
        } else {
            tilt_trigger_time_released_in_us += 1;
            tilt_trigger_time_pressed_in_us = 0;
        }
        if tilt_reset_pressed {
            tilt_reset_time_pressed_in_us += 1;
            tilt_reset_time_released_in_us = 0;
        } else {
            tilt_reset_time_released_in_us += 1;
            tilt_reset_time_pressed_in_us = 0;
        }

        if tilt_trigger_time_pressed_in_us >= tilt_trigger_time_limit_in_us {
            tilt_trigger_logically_pressed = true;
        } else if tilt_trigger_time_released_in_us >= tilt_trigger_time_limit_in_us {
            tilt_trigger_logically_pressed = false;
        }
        if tilt_reset_time_pressed_in_us >= tilt_reset_time_limit_in_us {
            tilt_reset_logically_pressed = true;
        } else if tilt_reset_time_released_in_us >= tilt_reset_time_limit_in_us {
            tilt_reset_logically_pressed = false;
        }

        if tilt_reset_logically_pressed || (tilt_trigger_logically_pressed && !needs_to_be_reset) {
            needs_to_be_reset = true;
            play_some_sound(loop_count, &snd_lib, &mut pico);
        }
        if tilt_reset_logically_pressed && needs_to_be_reset {
            needs_to_be_reset = false;
        }

        pico.set_led_state(pico::LedNames::Led1, tilt_reset_logically_pressed);
        pico.set_led_state(pico::LedNames::Led2, tilt_trigger_logically_pressed);

        // Loop is limited to 1us so we have semi-consistent timing
        pico.delay.delay_us(1);
    }
}

fn play_some_sound(loop_count: usize, snd_lib: &Library, pico: &mut Pico) {
    let metal_hit_key_pressed = pico.is_key_pressed(KeyNames::MetalHit);
    let body_hit_key_pressed = pico.is_key_pressed(KeyNames::BodyHit);

    if metal_hit_key_pressed {
        match loop_count % 2 {
            0 => pico.play_wav_blocking(&snd_lib.cbar_hit1),
            1 => pico.play_wav_blocking(&snd_lib.cbar_hit2),
            _ => {}
        };
    } else if body_hit_key_pressed {
        match loop_count % 3 {
            0 => pico.play_wav_blocking(&snd_lib.cbar_hitbod1),
            1 => pico.play_wav_blocking(&snd_lib.cbar_hitbod2),
            2 => pico.play_wav_blocking(&snd_lib.cbar_hitbod3),
            _ => {}
        }
    } else {
        pico.play_wav_blocking(&snd_lib.cbar_miss1);
    }
}

// const SAMPLE_RATE_22K: u32 = 22_050;
// const SAMPLE_WAIT_22K_IN_US: u32 = 1_000_000 / SAMPLE_RATE_22K;

// #[allow(dead_code)]
// const SAMPLE_RATE_11K: u32 = 11_025;
// #[allow(dead_code)]
// const SAMPLE_WAIT_11K_IN_US: u32 = 1_000_000 / SAMPLE_RATE_11K;

// fn play_8b_sound(pico: &mut Pico, sound: &[u8]) {
//     for i in 44..sound.len() {
//         pico.set_amplitude(sound[i]);

//         // Wait to maintain sample rate
//         pico.delay.delay_us(SAMPLE_WAIT_22K_IN_US);
//     }
// }

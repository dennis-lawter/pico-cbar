#![no_std]
#![no_main]

mod library;
mod pico;
mod wav;

use library::Library;
use pico::KeyNames;
use pico::Pico;

use library::*;

use rp_pico as bsp;

use bsp::entry;

#[entry]
fn main() -> ! {
    let mut pico = Pico::default();
    let snd_lib = Library::default();

    let mut tilt_sensor_time_pressed_in_us = 0usize;
    let tilt_sensor_time_limit_in_us = 10_000usize;

    let mut loop_count = 0;

    loop {
        loop_count += 1;
        let metal_hit_key_pressed = pico.is_key_pressed(KeyNames::MetalHit);
        let body_hit_key_pressed = pico.is_key_pressed(KeyNames::BodyHit);

        if tilt_sensor_time_pressed_in_us == tilt_sensor_time_limit_in_us {
            if metal_hit_key_pressed {
                match loop_count % 2 {
                    // 0 => play_8b_sound(&mut pico, CBAR_HIT1),
                    // 1 => play_8b_sound(&mut pico, CBAR_HIT2),
                    0 => pico.play_wav_blocking(&snd_lib.cbar_hit1),
                    1 => pico.play_wav_blocking(&snd_lib.cbar_hit2),
                    _ => {}
                };
            } else if body_hit_key_pressed {
                match loop_count % 3 {
                    // 0 => play_8b_sound(&mut pico, CBAR_HITBOD1),
                    // 1 => play_8b_sound(&mut pico, CBAR_HITBOD2),
                    // 2 => play_8b_sound(&mut pico, CBAR_HITBOD3),
                    0 => pico.play_wav_blocking(&snd_lib.cbar_hitbod1),
                    1 => pico.play_wav_blocking(&snd_lib.cbar_hitbod2),
                    2 => pico.play_wav_blocking(&snd_lib.cbar_hitbod3),
                    _ => {}
                }
            } else {
                // play_8b_sound(&mut pico, CBAR_MISS1);
                pico.play_wav_blocking(&snd_lib.cbar_miss1);
            }
        }

        if !pico.is_key_pressed(KeyNames::Tilt) && tilt_sensor_time_pressed_in_us < usize::MAX {
            tilt_sensor_time_pressed_in_us += 1;
        } else {
            tilt_sensor_time_pressed_in_us = 0;
        }

        // Slow it down, no need to busy loop
        pico.delay.delay_us(1);
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

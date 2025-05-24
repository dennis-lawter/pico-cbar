#![no_std]
#![no_main]

mod button;
// mod cbar; // working on rewrite to avoid the high amount of variable passing
mod library;
mod pico;
mod wav;

use button::Button;
use button::ButtonName;
use library::Library;
use pico::Pico;

use rp_pico as bsp;

use bsp::entry;

#[derive(PartialEq, Eq)]
pub enum CheatInputEvents {
    MetalDown,
    MetalUp,
    MeatDown,
    MeatUp,

    None,
}

#[entry]
fn main() -> ! {
    let mut pico = Pico::default();
    let snd_lib = Library::default();

    let mut metal_btn = Button::new(ButtonName::MetalHit);
    let mut meat_btn = Button::new(ButtonName::MeatHit);

    let mut trigger_btn = Button::new(ButtonName::TiltTrigger);
    let mut reset_btn = Button::new(ButtonName::TiltReset);

    let mut loop_count = 0usize;

    let mut primed = true;
    let us_required_for_valid_press = 10_000usize; // 1/100th of a second

    let mut cheat_recording: [CheatInputEvents; 16] = [const { CheatInputEvents::None }; 16];
    let cheat_valid_check = [
        // MM
        CheatInputEvents::MetalDown,
        CheatInputEvents::MetalUp,
        CheatInputEvents::MetalDown,
        CheatInputEvents::MetalUp,
        // BB
        CheatInputEvents::MeatDown,
        CheatInputEvents::MeatUp,
        CheatInputEvents::MeatDown,
        CheatInputEvents::MeatUp,
        // MB
        CheatInputEvents::MetalDown,
        CheatInputEvents::MetalUp,
        CheatInputEvents::MeatDown,
        CheatInputEvents::MeatUp,
        // MB
        CheatInputEvents::MetalDown,
        CheatInputEvents::MetalUp,
        CheatInputEvents::MeatDown,
        CheatInputEvents::MeatUp,
    ];
    let mut cheat_idx = 0usize;

    loop {
        loop_count = loop_count.wrapping_add(1);

        // Tick every button
        metal_btn.tick_us(&mut pico);
        meat_btn.tick_us(&mut pico);
        trigger_btn.tick_us(&mut pico);
        reset_btn.tick_us(&mut pico);

        // For debugging the tilt switches
        // green is upright, blue is down low
        pico.set_led_state(
            pico::LedNames::Led1,
            trigger_btn.is_button_held_past_limit(us_required_for_valid_press),
        );
        pico.set_led_state(
            pico::LedNames::Led2,
            reset_btn.is_button_held_past_limit(us_required_for_valid_press),
        );

        // For debugging wavs, can press both btns
        // if metal_btn.is_button_down() && meat_btn.is_button_down() {
        //     pico.play_wav_blocking(&snd_lib.uwish);
        // }

        if cheat_idx < cheat_recording.len() {
            if metal_btn.closed_us == us_required_for_valid_press {
                cheat_recording[cheat_idx] = CheatInputEvents::MetalDown;
                cheat_idx += 1;
            } else if metal_btn.open_us == us_required_for_valid_press {
                // This if fixes a special case...
                // When you first boot,
                // the metal_btn and meat_btn will both hit the OPEN timer
                // before a user can perform inputs
                if cheat_idx != 0 {
                    cheat_recording[cheat_idx] = CheatInputEvents::MetalUp;
                    cheat_idx += 1;
                }
            } else if meat_btn.closed_us == us_required_for_valid_press {
                cheat_recording[cheat_idx] = CheatInputEvents::MeatDown;
                cheat_idx += 1;
            } else if meat_btn.open_us == us_required_for_valid_press {
                cheat_recording[cheat_idx] = CheatInputEvents::MeatUp;
                cheat_idx += 1;
            }
        }

        if cheat_idx == cheat_recording.len() {
            let mut cheat_is_valid = true;
            for i in 0..cheat_recording.len() {
                if cheat_recording[i] != cheat_valid_check[i] {
                    cheat_is_valid = false;
                    break;
                }
            }
            if cheat_is_valid {
                pico.play_wav_blocking(&snd_lib.uwish);
                cheat_menu_loop();
            } else {
                pico.play_wav_blocking(&snd_lib.cbar_miss1);
                // Reset the cheat entries
                cheat_idx = 0;
                for i in 0..cheat_recording.len() {
                    cheat_recording[i] = CheatInputEvents::None;
                }
            }
        }

        if primed {
            if trigger_btn.is_button_held_past_limit(us_required_for_valid_press)
                && reset_btn.is_button_released_past_limit(us_required_for_valid_press)
            {
                // For debugging, LEDs both go on then off during sound playback
                pico.set_led_state(pico::LedNames::Led1, true);
                pico.set_led_state(pico::LedNames::Led2, true);
                play_some_sound(loop_count, &snd_lib, &mut pico, &metal_btn, &meat_btn);
                pico.set_led_state(pico::LedNames::Led1, false);
                pico.set_led_state(pico::LedNames::Led2, false);
                primed = false;
                // Reset the cheat entries
                cheat_idx = 0;
                for i in 0..cheat_recording.len() {
                    cheat_recording[i] = CheatInputEvents::None;
                }
            }
        } else {
            if reset_btn.is_button_held_past_limit(us_required_for_valid_press)
                && trigger_btn.is_button_released_past_limit(us_required_for_valid_press)
            {
                primed = true;
            }
        }

        pico.delay.delay_us(1);
    }
}

fn cheat_menu_loop() -> ! {
    loop {}
}

fn play_some_sound(
    loop_count: usize,
    snd_lib: &Library,
    pico: &mut Pico,
    metal_btn: &Button,
    meat_btn: &Button,
) {
    let metal_btn_down = metal_btn.is_button_down();
    let meat_btn_down = meat_btn.is_button_down();

    if metal_btn_down {
        match loop_count % 2 {
            0 => pico.play_wav_blocking(&snd_lib.cbar_hit1),
            1 => pico.play_wav_blocking(&snd_lib.cbar_hit2),
            _ => {}
        };
    } else if meat_btn_down {
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

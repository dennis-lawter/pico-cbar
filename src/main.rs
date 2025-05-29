#![no_std]
#![no_main]

mod button;
mod cbar;
mod library;
mod pico;
mod wav;

use cbar::Cbar;

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

enum States {
    Main,
    Cheat,
}

const US_REQUIRED_FOR_VALID_PRESS: usize = 10_000; // 1/100th of a second
const CHEAT_CODE: [CheatInputEvents; 16] = [
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

#[entry]
fn main() -> ! {
    let mut cbar = Cbar::default();

    let mut cheat_recording: [CheatInputEvents; 16] = [const { CheatInputEvents::None }; 16];
    let mut cheat_idx = 0usize;
    let mut state = States::Main;

    loop {
        state = match state {
            States::Main => main_state_loop(&mut cbar, &mut cheat_recording, &mut cheat_idx),
            States::Cheat => cheat_state_loop(&mut cbar),
        };
    }
}

fn main_state_loop(
    cbar: &mut Cbar<'static>,
    cheat_recording: &mut [CheatInputEvents],
    cheat_idx: &mut usize,
) -> States {
    cbar.tick();
    if *cheat_idx == cheat_recording.len() {
        let mut cheat_is_valid = true;
        for i in 0..cheat_recording.len() {
            if cheat_recording[i] != CHEAT_CODE[i] {
                cheat_is_valid = false;
                break;
            }
        }
        if cheat_is_valid {
            cbar.pico.play_wav_blocking(&cbar.snd_lib.uwish);
            return States::Cheat;
        } else {
            cbar.pico.play_wav_blocking(&cbar.snd_lib.cbar_miss1);
            // Reset the cheat entries
            *cheat_idx = 0;
            for i in 0..cheat_recording.len() {
                cheat_recording[i] = CheatInputEvents::None;
            }
        }
    } else if *cheat_idx < cheat_recording.len() {
        if cbar.metal_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
            cheat_recording[*cheat_idx] = CheatInputEvents::MetalDown;
            *cheat_idx += 1;
        } else if cbar.metal_btn.open_us == US_REQUIRED_FOR_VALID_PRESS {
            // Can't start a cheat with a button up
            if *cheat_idx != 0 {
                cheat_recording[*cheat_idx] = CheatInputEvents::MetalUp;
                *cheat_idx += 1;
            }
        } else if cbar.body_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
            cheat_recording[*cheat_idx] = CheatInputEvents::MeatDown;
            *cheat_idx += 1;
        } else if cbar.body_btn.open_us == US_REQUIRED_FOR_VALID_PRESS {
            // Can't start a cheat with a button up
            if *cheat_idx != 0 {
                cheat_recording[*cheat_idx] = CheatInputEvents::MeatUp;
                *cheat_idx += 1;
            }
        }
    }

    if cbar.swing_primed {
        if cbar
            .trigger_btn
            .is_button_held_past_limit(US_REQUIRED_FOR_VALID_PRESS)
            && cbar
                .reset_btn
                .is_button_released_past_limit(US_REQUIRED_FOR_VALID_PRESS)
        {
            // For debugging, LEDs both go on then off during sound playback
            cbar.pico.set_led_state(pico::LedNames::Led1, true);
            cbar.pico.set_led_state(pico::LedNames::Led2, true);
            play_some_sound(cbar);
            cbar.pico.set_led_state(pico::LedNames::Led1, false);
            cbar.pico.set_led_state(pico::LedNames::Led2, false);
            cbar.swing_primed = false;
            // Reset the cheat entries
            *cheat_idx = 0;
            for i in 0..cheat_recording.len() {
                cheat_recording[i] = CheatInputEvents::None;
            }
        }
    } else {
        if cbar
            .reset_btn
            .is_button_held_past_limit(US_REQUIRED_FOR_VALID_PRESS)
            && cbar
                .trigger_btn
                .is_button_released_past_limit(US_REQUIRED_FOR_VALID_PRESS)
        {
            cbar.swing_primed = true;
        }
    }

    States::Main
}

fn cheat_state_loop(_cbar: &mut Cbar) -> States {
    States::Cheat
}

fn play_some_sound(cbar: &mut Cbar) {
    let metal_btn_down = cbar.metal_btn.is_button_down();
    let body_btn_down = cbar.body_btn.is_button_down();

    if metal_btn_down {
        match cbar.loop_count % 2 {
            0 => cbar.pico.play_wav_blocking(&cbar.snd_lib.cbar_hit1),
            1 => cbar.pico.play_wav_blocking(&cbar.snd_lib.cbar_hit2),
            _ => {}
        };
    } else if body_btn_down {
        match cbar.loop_count % 3 {
            0 => cbar.pico.play_wav_blocking(&cbar.snd_lib.cbar_hitbod1),
            1 => cbar.pico.play_wav_blocking(&cbar.snd_lib.cbar_hitbod2),
            2 => cbar.pico.play_wav_blocking(&cbar.snd_lib.cbar_hitbod3),
            _ => {}
        }
    } else {
        cbar.pico.play_wav_blocking(&cbar.snd_lib.cbar_miss1);
    }
}

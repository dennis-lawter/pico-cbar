#![no_std]
#![no_main]

mod button;
mod cbar;
mod cheat_code;
mod cheat_soundboard;
mod pico;
mod sound_library;
mod wav;

use cbar::Cbar;

use cheat_code::CheatCodeRecord;
use cheat_code::CheatInputEvents;
use rp_pico as bsp;

use bsp::entry;

const US_REQUIRED_FOR_VALID_PRESS: usize = 10_000; // 1/100th of a second

enum States {
    Main,
    Cheat,
}

#[entry]
fn main() -> ! {
    let mut cbar = Cbar::default();
    let mut cheat_code_record = CheatCodeRecord::default();

    // let mut cheat_recording: [CheatInputEvents; 16] = [const { CheatInputEvents::None }; 16];
    // let mut cheat_idx = 0usize;
    let mut state = States::Main;

    loop {
        state = match state {
            States::Main => main_state_loop(&mut cbar, &mut cheat_code_record),
            States::Cheat => cheat_state_loop(&mut cbar),
        };
    }
}

fn main_state_loop(cbar: &mut Cbar<'static>, cheat_code: &mut CheatCodeRecord) -> States {
    cbar.tick();

    match cheat_code.validate() {
        Some(true) => {
            cbar.pico.play_wav_blocking(&cbar.snd_lib.uwish);
            return States::Cheat;
        }
        Some(false) => {
            cbar.pico.play_wav_blocking(&cbar.snd_lib.cbar_miss1);
        }
        None => {}
    }

    if cbar.metal_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::MetalDown);
    } else if cbar.metal_btn.open_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::MetalUp);
    }
    if cbar.body_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::BodyDown);
    } else if cbar.body_btn.open_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::BodyUp);
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

            cheat_code.reset();
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

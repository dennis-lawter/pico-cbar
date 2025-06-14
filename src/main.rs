#![no_std]
#![no_main]

mod button;
mod cbar;
mod cheat_code;
mod pico;
mod player;
mod sound_library;
mod soundboard;
mod wav;

use cbar::Cbar;

use cheat_code::CheatCodeRecord;
use cheat_code::CheatInputEvents;
use rp_pico as bsp;

use bsp::entry;

use crate::cheat_code::CHEAT_CODE;
use crate::wav::Wav;

const US_REQUIRED_FOR_VALID_PRESS: usize = 10_000; // 1/100th of a second

enum States {
    Main,
    Cheat,
}

#[entry]
fn main() -> ! {
    let mut cbar = Cbar::default();
    unsafe {
        player::PICO = Some(&mut *&mut cbar.pico);
    }
    let mut cheat_code_record = CheatCodeRecord::default();

    let mut state = States::Main;

    loop {
        state = match state {
            States::Main => main_state_loop(&mut cbar, &mut cheat_code_record),
            States::Cheat => cheat_state_loop(&mut cbar, &mut cheat_code_record),
        };
    }
}

fn main_state_loop(cbar: &mut Cbar<'static>, cheat_code: &mut CheatCodeRecord) -> States {
    cbar.tick();

    match cheat_code.validate(&CHEAT_CODE) {
        Some(true) => {
            let wavdat = cbar.soundboard.cheat_entry.get_by_index(1);
            let wav = Wav::new(wavdat);
            cbar.pico.play_wav_blocking(&wav);
            cheat_code.reset();
            return States::Cheat;
        }
        Some(false) => {
            let wavdat = cbar.soundboard.cheat_entry.get_by_index(0);
            let wav = Wav::new(wavdat);
            cbar.pico.play_wav_blocking(&wav);
            cheat_code.reset();
        }
        None => {}
    }

    if cbar.metal_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::MetalDown);
    }
    if cbar.body_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::BodyDown);
    }

    let trigger_valid_press = cbar
        .trigger_btn
        .is_button_held_past_limit(US_REQUIRED_FOR_VALID_PRESS);
    let trigger_valid_release = cbar
        .trigger_btn
        .is_button_released_past_limit(US_REQUIRED_FOR_VALID_PRESS);
    let reset_valid_press = cbar
        .reset_btn
        .is_button_held_past_limit(US_REQUIRED_FOR_VALID_PRESS);
    let reset_valid_release = cbar
        .reset_btn
        .is_button_released_past_limit(US_REQUIRED_FOR_VALID_PRESS);

    if cbar.swing_primed && trigger_valid_press && reset_valid_release {
        let wavdata = if cbar.metal_btn.is_button_down() {
            cbar.soundboard.cbar_metal.get_by_index(cbar.loop_count)
        } else if cbar.body_btn.is_button_down() {
            cbar.soundboard.cbar_body.get_by_index(cbar.loop_count)
        } else {
            cbar.soundboard.cbar_miss.get_by_index(cbar.loop_count)
        };

        let wav = Wav::new(wavdata);
        cbar.pico.play_wav_blocking(&wav);
        cbar.swing_primed = false;

        cheat_code.reset();
    } else if reset_valid_press && trigger_valid_release {
        cbar.swing_primed = true;
    }

    States::Main
}

fn cheat_state_loop(cbar: &mut Cbar<'static>, cheat_code: &mut CheatCodeRecord) -> States {
    cbar.tick();
    let led_state = (cbar.loop_count / 100_000) % 2 == 0;
    cbar.pico.set_led_state(led_state);

    if cbar.metal_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::MetalDown);
    }
    if cbar.body_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::BodyDown);
    }

    match cheat_code.validate(&cheat_code::BARNEY_SMELL) {
        None => {}
        Some(true) => {
            cbar.pico.set_led_state(true);
            let test = cbar.soundboard.ba_smell.get_next();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            cheat_code.reset();
        }
        Some(false) => {}
    }

    match cheat_code.validate(&cheat_code::BARNEY_DRINK) {
        None => {}
        Some(true) => {
            cbar.pico.set_led_state(true);
            let test = cbar.soundboard.ba_drink.get_next();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            cheat_code.reset();
        }
        Some(false) => {}
    }

    match cheat_code.validate(&cheat_code::BARNEY_FUNNY) {
        None => {}
        Some(true) => {
            cbar.pico.set_led_state(true);
            let test = cbar.soundboard.ba_funny.get_next();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            cheat_code.reset();
        }
        Some(false) => {}
    }

    match cheat_code.validate(&cheat_code::BARNEY_SCREAM) {
        None => {}
        Some(true) => {
            cbar.pico.set_led_state(true);
            let test = cbar.soundboard.ba_scream.get_next();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            cheat_code.reset();
        }
        Some(false) => {}
    }

    match cheat_code.validate(&cheat_code::SCIENTIST_SMELL) {
        None => {}
        Some(true) => {
            cbar.pico.set_led_state(true);
            let test = cbar.soundboard.sci_smell.get_next();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            cheat_code.reset();
        }
        Some(false) => {}
    }

    match cheat_code.validate(&cheat_code::SCIENTIST_DRINK) {
        None => {}
        Some(true) => {
            cbar.pico.set_led_state(true);
            let test = cbar.soundboard.sci_drink.get_next();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            cheat_code.reset();
        }
        Some(false) => {}
    }

    match cheat_code.validate(&cheat_code::SCIENTIST_FUNNY) {
        None => {}
        Some(true) => {
            cbar.pico.set_led_state(true);
            let test = cbar.soundboard.sci_funny.get_next();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            cheat_code.reset();
        }
        Some(false) => {}
    }

    match cheat_code.validate(&cheat_code::SCIENTIST_SCREAM) {
        None => {}
        Some(true) => {
            cbar.pico.set_led_state(true);
            let test = cbar.soundboard.sci_scream.get_next();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            cheat_code.reset();
        }
        Some(false) => {}
    }

    States::Cheat
}

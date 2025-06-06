#![no_std]
#![no_main]

mod button;
mod cbar;
mod cheat_code;
mod pico;
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
            // cbar.pico.play_wav_blocking(&cbar.soundboard.snd_lib.uwish);
            cheat_code.reset();
            return States::Cheat;
        }
        Some(false) => {
            // cbar.pico.play_wav_blocking(&cbar.soundboard.snd_lib.wrong);
        }
        None => {}
    }

    if cbar.metal_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::MetalDown);
    }
    if cbar.body_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::BodyDown);
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

            // let wav = if cbar.metal_btn.is_button_down() {
            //     cbar.soundboard.cbar_metal.get_rand(cbar.loop_count)
            // } else if cbar.body_btn.is_button_down() {
            //     cbar.soundboard.cbar_body.get_rand(cbar.loop_count)
            // } else {
            //     cbar.soundboard.cbar_miss.get_rand(cbar.loop_count)
            // };

            // cbar.pico.play_wav_blocking(&wav);

            // play_some_sound(cbar);
            let wavdat = cbar.soundboard.ba_drink.get_rand(cbar.loop_count);
            let wav = Wav::new(wavdat);
            cbar.pico.play_wav_blocking(&wav);
            // cbar.pico.delay.delay_ms(1000);
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

fn cheat_state_loop(cbar: &mut Cbar<'static>, cheat_code: &mut CheatCodeRecord) -> States {
    cbar.tick();

    if cbar.metal_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::MetalDown);
    }
    if cbar.body_btn.closed_us == US_REQUIRED_FOR_VALID_PRESS {
        cheat_code.add_event(CheatInputEvents::BodyDown);
    }

    match cheat_code.validate(&cheat_code::BARNEY_SMELL) {
        None => {}
        Some(true) => {
            // let test = cbar.soundboard.sci_drink.sounds[0].clone().unwrap().clone();
            let test = cbar.soundboard.sci_drink.get_next().clone();
            let test_wav = Wav::new(test);
            cbar.pico.play_wav_blocking(&test_wav);
            // let sb = &mut cbar.soundboard;
            // let pico = &mut cbar.pico;
            // sb.ba_smell.play_next(pico);
            // cbar.soundboard.ba_smell.play_next(&mut cbar.pico);
            cheat_code.reset();
        }
        Some(false) => {
            // cbar.pico
            //     .play_wav_blocking(&cbar.soundboard.snd_lib.ba_beertopside);
            cheat_code.reset();
        }
    }

    match cheat_code.validate(&cheat_code::BARNEY_DRINK) {
        None => {}
        Some(true) => {}
        Some(false) => {}
    }

    States::Cheat
}

// fn play_some_sound(cbar: &mut Cbar) {
//     let metal_btn_down = cbar.metal_btn.is_button_down();
//     let body_btn_down = cbar.body_btn.is_button_down();

//     /*let wav = */
//     if metal_btn_down {
//         // cbar.soundboard.cbar_metal.get_rand(cbar.loop_count)
//         match cbar.loop_count % 2 {
//             0 => cbar
//                 .pico
//                 .play_wav_blocking(&cbar.soundboard.snd_lib.cbar_hit1),
//             1 => cbar
//                 .pico
//                 .play_wav_blocking(&cbar.soundboard.snd_lib.cbar_hit2),
//             _ => {}
//         };
//     } else if body_btn_down {
//         // cbar.soundboard.cbar_body.get_rand(cbar.loop_count)
//         match cbar.loop_count % 3 {
//             0 => cbar
//                 .pico
//                 .play_wav_blocking(&cbar.soundboard.snd_lib.cbar_hitbod1),
//             1 => cbar
//                 .pico
//                 .play_wav_blocking(&cbar.soundboard.snd_lib.cbar_hitbod2),
//             2 => cbar
//                 .pico
//                 .play_wav_blocking(&cbar.soundboard.snd_lib.cbar_hitbod3),
//             _ => {}
//         }
//     } else {
//         // cbar.soundboard.cbar_miss.get_rand(cbar.loop_count)
//         cbar.pico
//             .play_wav_blocking(&cbar.soundboard.snd_lib.cbar_miss1);
//     } //;

//     // cbar.pico.play_wav_blocking(&wav)
// }

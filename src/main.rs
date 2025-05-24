#![no_std]
#![no_main]

mod button;
mod library;
mod pico;
mod wav;

use button::Button;
use button::ButtonName;
use library::Library;
use pico::Pico;

use rp_pico as bsp;

use bsp::entry;

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

    loop {
        loop_count = loop_count.wrapping_add(1);

        metal_btn.tick_us(&mut pico);
        meat_btn.tick_us(&mut pico);
        trigger_btn.tick_us(&mut pico);
        reset_btn.tick_us(&mut pico);

        pico.set_led_state(
            pico::LedNames::Led1,
            trigger_btn.is_button_held_past_limit(us_required_for_valid_press),
        );
        pico.set_led_state(
            pico::LedNames::Led2,
            reset_btn.is_button_held_past_limit(us_required_for_valid_press),
        );

        if primed {
            if trigger_btn.is_button_held_past_limit(us_required_for_valid_press)
                && reset_btn.is_button_released_past_limit(us_required_for_valid_press)
            {
                pico.set_led_state(pico::LedNames::Led1, true);
                pico.set_led_state(pico::LedNames::Led2, true);
                play_some_sound(loop_count, &snd_lib, &mut pico, &metal_btn, &meat_btn);
                pico.set_led_state(pico::LedNames::Led1, false);
                pico.set_led_state(pico::LedNames::Led2, false);
                primed = false;
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

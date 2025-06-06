use crate::button::Button;
use crate::button::ButtonName;
// use crate::cheat_soundboard;
use crate::pico::Pico;
use crate::soundboard::Soundboard;
// use crate::sound_library::SoundLibrary;
// use crate::library::Library;
// use crate::pico;
// use crate::pico::Pico;

pub struct Cbar<'a> {
    pub pico: Pico,
    pub metal_btn: Button,
    pub body_btn: Button,
    pub trigger_btn: Button,
    pub reset_btn: Button,
    // pub snd_lib: SoundLibrary<'a>,
    pub loop_count: usize,
    pub swing_primed: bool,
    pub soundboard: Soundboard<'a>,
}
impl Default for Cbar<'static> {
    fn default() -> Self {
        let pico = Pico::default();
        let metal_btn = Button::new(ButtonName::MetalHit);
        let body_btn = Button::new(ButtonName::BodyHit);

        let trigger_btn = Button::new(ButtonName::TiltTrigger);
        let reset_btn = Button::new(ButtonName::TiltReset);

        // let snd_lib = SoundLibrary::default();

        let soundboard = Soundboard::new();

        let loop_count = 0usize;
        let swing_primed = true;

        Self {
            pico,
            metal_btn,
            body_btn,
            trigger_btn,
            reset_btn,
            // snd_lib,
            loop_count,
            swing_primed,
            soundboard,
        }
    }
}
impl Cbar<'static> {
    pub fn tick(&mut self) {
        self.pico.delay.delay_us(1);
        self.loop_count = self.loop_count.wrapping_add(1);

        self.metal_btn.tick_us(&mut self.pico);
        self.body_btn.tick_us(&mut self.pico);
        self.trigger_btn.tick_us(&mut self.pico);
        self.reset_btn.tick_us(&mut self.pico);
    }
}

// pub struct Cbar<'a> {
//     pico: Pico,
//     snd_lib: Library<'a>,
//     metal_btn: Button,
//     meat_btn: Button,
//     trigger_btn: Button,
//     reset_btn: Button,
// }
// impl Cbar<'static> {
//     pub fn main_loop(&mut self) -> ! {
//         let mut loop_count = 0usize;

//         let mut primed = true;
//         let us_required_for_valid_press = 10_000usize; // 1/100th of a second

//         let mut cheat_recording: [CheatInputEvents; 16] = [const { CheatInputEvents::None }; 16];
//         // MMBBMBMB is the cheat code. M = metal, B = body
//         let cheat_valid_check = [
//             // MM
//             CheatInputEvents::MetalDown,
//             CheatInputEvents::MetalUp,
//             CheatInputEvents::MetalDown,
//             CheatInputEvents::MetalUp,
//             // BB
//             CheatInputEvents::MeatDown,
//             CheatInputEvents::MeatUp,
//             CheatInputEvents::MeatDown,
//             CheatInputEvents::MeatUp,
//             // MB
//             CheatInputEvents::MetalDown,
//             CheatInputEvents::MetalUp,
//             CheatInputEvents::MeatDown,
//             CheatInputEvents::MeatUp,
//             // MB
//             CheatInputEvents::MetalDown,
//             CheatInputEvents::MetalUp,
//             CheatInputEvents::MeatDown,
//             CheatInputEvents::MeatUp,
//         ];
//         let mut cheat_idx = 0usize;

//         loop {
//             loop_count = loop_count.wrapping_add(1);

//             // Tick every button
//             self.metal_btn.tick_us(&mut self.pico);
//             self.meat_btn.tick_us(&mut self.pico);
//             self.trigger_btn.tick_us(&mut self.pico);
//             self.reset_btn.tick_us(&mut self.pico);

//             // For debugging the tilt switches
//             // green is upright, blue is down low
//             self.pico.set_led_state(
//                 pico::LedNames::Led1,
//                 self.trigger_btn
//                     .is_button_held_past_limit(us_required_for_valid_press),
//             );
//             self.pico.set_led_state(
//                 pico::LedNames::Led2,
//                 self.reset_btn
//                     .is_button_held_past_limit(us_required_for_valid_press),
//             );

//             // For debugging wavs, can press both btns
//             // if metal_btn.is_button_down() && meat_btn.is_button_down() {
//             //     self.pico.play_wav_blocking(&self.snd_lib.uwish);
//             // }

//             if cheat_idx < cheat_recording.len() {
//                 if self.metal_btn.closed_us == us_required_for_valid_press {
//                     cheat_recording[cheat_idx] = CheatInputEvents::MetalDown;
//                     cheat_idx += 1;
//                 } else if self.metal_btn.open_us == us_required_for_valid_press {
//                     // This if fixes a special case...
//                     // When you first boot,
//                     // the metal_btn and meat_btn will both hit the OPEN timer
//                     // before a user can perform inputs
//                     if cheat_idx != 0 {
//                         cheat_recording[cheat_idx] = CheatInputEvents::MetalUp;
//                         cheat_idx += 1;
//                     }
//                 } else if self.meat_btn.closed_us == us_required_for_valid_press {
//                     cheat_recording[cheat_idx] = CheatInputEvents::MeatDown;
//                     cheat_idx += 1;
//                 } else if self.meat_btn.open_us == us_required_for_valid_press {
//                     cheat_recording[cheat_idx] = CheatInputEvents::MeatUp;
//                     cheat_idx += 1;
//                 }
//             }

//             if cheat_idx == cheat_recording.len() {
//                 let mut cheat_is_valid = true;
//                 for i in 0..cheat_recording.len() {
//                     if cheat_recording[i] != cheat_valid_check[i] {
//                         cheat_is_valid = false;
//                         break;
//                     }
//                 }
//                 if cheat_is_valid {
//                     self.pico.play_wav_blocking(&self.snd_lib.uwish);
//                     self.cheat_menu_loop();
//                 } else {
//                     self.pico.play_wav_blocking(&self.snd_lib.cbar_miss1);
//                     // Reset the cheat entries
//                     cheat_idx = 0;
//                     for i in 0..cheat_recording.len() {
//                         cheat_recording[i] = CheatInputEvents::None;
//                     }
//                 }
//             }

//             if primed {
//                 if self
//                     .trigger_btn
//                     .is_button_held_past_limit(us_required_for_valid_press)
//                     && self
//                         .reset_btn
//                         .is_button_released_past_limit(us_required_for_valid_press)
//                 {
//                     // For debugging, LEDs both go on then off during sound playback
//                     self.pico.set_led_state(pico::LedNames::Led1, true);
//                     self.pico.set_led_state(pico::LedNames::Led2, true);
//                     self.play_some_sound(loop_count);
//                     self.pico.set_led_state(pico::LedNames::Led1, false);
//                     self.pico.set_led_state(pico::LedNames::Led2, false);
//                     primed = false;
//                     // Reset the cheat entries
//                     cheat_idx = 0;
//                     for i in 0..cheat_recording.len() {
//                         cheat_recording[i] = CheatInputEvents::None;
//                     }
//                 }
//             } else {
//                 if self
//                     .reset_btn
//                     .is_button_held_past_limit(us_required_for_valid_press)
//                     && self
//                         .trigger_btn
//                         .is_button_released_past_limit(us_required_for_valid_press)
//                 {
//                     primed = true;
//                 }
//             }

//             self.pico.delay.delay_us(1);
//         }
//     }
// }
// impl Default for Cbar<'static> {
//     fn default() -> Self {
//         let pico = Pico::default();
//         let snd_lib = Library::default();

//         let metal_btn = Button::new(ButtonName::MetalHit);
//         let meat_btn = Button::new(ButtonName::MeatHit);

//         let trigger_btn = Button::new(ButtonName::TiltTrigger);
//         let reset_btn = Button::new(ButtonName::TiltReset);
//         Self {
//             pico,
//             snd_lib,
//             metal_btn,
//             meat_btn,
//             trigger_btn,
//             reset_btn,
//         }
//     }
// }

// impl Cbar<'static> {
//     fn cheat_menu_loop(&mut self) -> ! {
//         loop {}
//     }
//     fn play_some_sound(&mut self, loop_count: usize) {
//         let metal_btn_down = self.metal_btn.is_button_down();
//         let meat_btn_down = self.meat_btn.is_button_down();

//         if metal_btn_down {
//             match loop_count % 2 {
//                 0 => self.pico.play_wav_blocking(&self.snd_lib.cbar_hit1),
//                 1 => self.pico.play_wav_blocking(&self.snd_lib.cbar_hit2),
//                 _ => {}
//             };
//         } else if meat_btn_down {
//             match loop_count % 3 {
//                 0 => self.pico.play_wav_blocking(&self.snd_lib.cbar_hitbod1),
//                 1 => self.pico.play_wav_blocking(&self.snd_lib.cbar_hitbod2),
//                 2 => self.pico.play_wav_blocking(&self.snd_lib.cbar_hitbod3),
//                 _ => {}
//             }
//         } else {
//             self.pico.play_wav_blocking(&self.snd_lib.cbar_miss1);
//         }
//     }
// }

// #[derive(PartialEq, Eq)]
// pub enum CheatInputEvents {
//     MetalDown,
//     MetalUp,
//     MeatDown,
//     MeatUp,

//     None,
// }

use crate::button::Button;
use crate::button::ButtonName;
use crate::pico::Pico;
use crate::soundboard::Soundboard;

pub struct Cbar<'a> {
    pub pico: Pico,
    pub metal_btn: Button,
    pub body_btn: Button,
    pub trigger_btn: Button,
    pub reset_btn: Button,
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

        let soundboard = Soundboard::new();

        let loop_count = 0usize;
        let swing_primed = true;

        Self {
            pico,
            metal_btn,
            body_btn,
            trigger_btn,
            reset_btn,
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

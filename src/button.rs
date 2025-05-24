use crate::pico::Pico;

#[derive(Clone, Copy)]
pub enum ButtonName {
    TiltTrigger,
    TiltReset,
    MetalHit,
    MeatHit,
}
pub struct Button {
    pub name: ButtonName,
    pub closed_us: usize,
    pub open_us: usize,
    pub just_pressed: bool,
    pub just_released: bool,
}
impl Button {
    pub fn new(name: ButtonName) -> Self {
        Self {
            name,
            closed_us: 0usize,
            open_us: 0usize,
            just_pressed: false,
            just_released: false,
        }
    }

    pub fn tick_us(&mut self, pico: &mut Pico) {
        self.just_pressed = false;
        self.just_released = false;
        let is_button_closed = pico.is_button_pressed(self.name);
        if is_button_closed {
            if self.closed_us == 0 {
                self.just_pressed = true;
            }
            self.closed_us = self.closed_us.saturating_add(1);
            self.open_us = 0;
        } else {
            if self.open_us == 0 {
                self.just_released = true;
            }
            self.open_us = self.open_us.saturating_add(1);
            self.closed_us = 0;
        }
    }

    pub fn is_button_held_past_limit(&self, limit: usize) -> bool {
        self.closed_us >= limit
    }

    pub fn is_button_released_past_limit(&self, limit: usize) -> bool {
        self.open_us >= limit
    }

    pub fn is_button_down(&self) -> bool {
        self.closed_us > 0
    }

    // pub fn is_button_up(&self) -> bool {
    //     self.open_us > 0
    // }
}

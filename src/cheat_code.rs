pub struct CheatCodeRecord {
    record: [CheatInputEvents; 16],
    idx: usize,
}
impl Default for CheatCodeRecord {
    fn default() -> Self {
        let record = [const { CheatInputEvents::None }; 16];
        Self { record, idx: 0 }
    }
}
impl CheatCodeRecord {
    pub fn add_event(&mut self, event: CheatInputEvents) {
        if self.idx >= self.record.len() {
            // Guard: don't write past the buffer's end
            return;
        }
        if self.idx == 0 && event.is_release_type() {
            // Guard: don't start recording on a button release
            return;
        }
        self.record[self.idx] = event;
        self.idx += 1;
    }
    pub fn validate(&mut self) -> Option<bool> {
        if self.idx == self.record.len() {
            for i in 0..self.record.len() {
                if self.record[i] != CHEAT_CODE[i] {
                    self.reset();
                    return Some(false);
                }
            }
            return Some(true);
        }
        None
    }

    pub fn reset(&mut self) {
        self.idx = 0;
        for i in 0..self.record.len() {
            self.record[i] = CheatInputEvents::None;
        }
    }
}

#[derive(PartialEq, Eq)]
pub enum CheatInputEvents {
    MetalDown,
    MetalUp,
    BodyDown,
    BodyUp,

    None,
}
impl CheatInputEvents {
    fn is_release_type(&self) -> bool {
        match self {
            CheatInputEvents::MetalUp | CheatInputEvents::BodyUp => true,
            _ => false,
        }
    }
}
const CHEAT_CODE: [CheatInputEvents; 16] = [
    // MM
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalUp,
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalUp,
    // BB
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyUp,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyUp,
    // MB
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalUp,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyUp,
    // MB
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalUp,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyUp,
];

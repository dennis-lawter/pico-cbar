pub struct CheatCodeRecord {
    record: [CheatInputEvents; 8],
    pub idx: usize,
}
impl Default for CheatCodeRecord {
    fn default() -> Self {
        let record = [const { CheatInputEvents::None }; 8];
        Self { record, idx: 0 }
    }
}
impl CheatCodeRecord {
    pub fn add_event(&mut self, event: CheatInputEvents) {
        if self.idx >= self.record.len() {
            return;
        }
        self.record[self.idx] = event;
        self.idx += 1;
    }
    pub fn validate(&mut self, pattern: &[CheatInputEvents]) -> Option<bool> {
        if self.idx == pattern.len() {
            for i in 0..pattern.len() {
                if self.record[i] != pattern[i] {
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
    BodyDown,

    None,
}

pub const CHEAT_CODE: [CheatInputEvents; 8] = [
    // MMBB
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    // MBMB
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
];

pub const BARNEY_SMELL: [CheatInputEvents; 2] = [
    // MB
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
];

pub const BARNEY_DRINK: [CheatInputEvents; 3] = [
    // MMB
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
];

pub const BARNEY_FUNNY: [CheatInputEvents; 4] = [
    // MMMB
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
];

pub const BARNEY_SCREAM: [CheatInputEvents; 4] = [
    // MMMM
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
];

pub const SCIENTIST_SMELL: [CheatInputEvents; 2] = [
    // BM
    CheatInputEvents::BodyDown,
    CheatInputEvents::MetalDown,
];

pub const SCIENTIST_DRINK: [CheatInputEvents; 3] = [
    // BBM
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::MetalDown,
];

pub const SCIENTIST_FUNNY: [CheatInputEvents; 4] = [
    // BBBM
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::MetalDown,
];

pub const SCIENTIST_SCREAM: [CheatInputEvents; 4] = [
    // BBBB
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
];

pub struct CheatCodeRecord {
    record: [CheatInputEvents; 8],
    idx: usize,
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
    BodyDown,

    None,
}

const CHEAT_CODE: [CheatInputEvents; 8] = [
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

const BARNEY_SMELL: [CheatInputEvents; 2] = [
    // MB
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
];

const BARNEY_DRINK: [CheatInputEvents; 3] = [
    // MMB
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
];

const BARNEY_FUNNY: [CheatInputEvents; 4] = [
    // MMMB
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::BodyDown,
];

const BARNEY_DRINK: [CheatInputEvents; 4] = [
    // MMMM
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
    CheatInputEvents::MetalDown,
];

const SCIENTIST_SMELL: [CheatInputEvents; 2] = [
    // BM
    CheatInputEvents::BodyDown,
    CheatInputEvents::MetalDown,
];

const SCIENTIST_DRINK: [CheatInputEvents; 3] = [
    // BBM
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::MetalDown,
];

const SCIENTIST_FUNNY: [CheatInputEvents; 4] = [
    // BBBM
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::MetalDown,
];

const SCIENTIST_DRINK: [CheatInputEvents; 4] = [
    // BBBB
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
    CheatInputEvents::BodyDown,
];

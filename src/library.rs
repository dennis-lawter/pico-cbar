use crate::wav::Wav;

const CBAR_HIT1: &[u8; 7784] = include_bytes!("../sfx/cbar_hit1.wav");
const CBAR_HIT2: &[u8; 5530] = include_bytes!("../sfx/cbar_hit2.wav");

const CBAR_HITBOD1: &[u8; 7048] = include_bytes!("../sfx/cbar_hitbod1.wav");
const CBAR_HITBOD2: &[u8; 5518] = include_bytes!("../sfx/cbar_hitbod2.wav");
const CBAR_HITBOD3: &[u8; 7744] = include_bytes!("../sfx/cbar_hitbod3.wav");

const CBAR_MISS1: &[u8; 7710] = include_bytes!("../sfx/cbar_miss1.wav");

const UWISH: &[u8; 30048] = include_bytes!("../sfx/ba_uwish.wav");

pub struct Library<'a> {
    pub cbar_hit1: Wav<'a>,
    pub cbar_hit2: Wav<'a>,

    pub cbar_hitbod1: Wav<'a>,
    pub cbar_hitbod2: Wav<'a>,
    pub cbar_hitbod3: Wav<'a>,

    pub cbar_miss1: Wav<'a>,

    pub uwish: Wav<'a>,
}
impl Default for Library<'static> {
    fn default() -> Self {
        Self {
            cbar_hit1: Wav::new(CBAR_HIT1),
            cbar_hit2: Wav::new(CBAR_HIT2),
            cbar_hitbod1: Wav::new(CBAR_HITBOD1),
            cbar_hitbod2: Wav::new(CBAR_HITBOD2),
            cbar_hitbod3: Wav::new(CBAR_HITBOD3),
            cbar_miss1: Wav::new(CBAR_MISS1),
            uwish: Wav::new(UWISH),
        }
    }
}

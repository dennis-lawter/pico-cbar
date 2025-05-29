use crate::sound_library::SoundLibrary;
use crate::wav::Wav;

pub struct Playlist<'a> {
    index: u8,
    sounds: [Option<&'a Wav<'a>>; 16],
    length: u8,
}
impl Playlist<'static> {
    fn new() -> Self {
        let sounds = [None; 16];
        Self {
            index: 0u8,
            sounds,
            length: 0u8,
        }
    }

    fn insert(&mut self, sound: &'static Wav) -> Result<(), ()> {
        if self.length >= 16 {
            return Err(());
        }
        self.sounds[self.length as usize] = Some(sound);
        Ok(())
    }
}

pub struct Soundboard<'a> {
    pub ba_smell: Playlist<'a>,
    pub ba_drink: Playlist<'a>,
    pub ba_funny: Playlist<'a>,
    pub ba_scream: Playlist<'a>,

    pub sci_smell: Playlist<'a>,
    pub sci_drink: Playlist<'a>,
    pub sci_funny: Playlist<'a>,
    pub sci_scream: Playlist<'a>,
}
impl Soundboard<'static> {
    fn new(snd_lib: &SoundLibrary) -> Self {
        let mut ba_smell = Playlist::new();
        // ba_smell.insert()
        let mut ba_drink = Playlist::new();
        let mut ba_funny = Playlist::new();
        let mut ba_scream = Playlist::new();
        let mut sci_smell = Playlist::new();
        let mut sci_drink = Playlist::new();
        let mut sci_funny = Playlist::new();
        let mut sci_scream = Playlist::new();
        Self {
            ba_smell,
            ba_drink,
            ba_funny,
            ba_scream,
            sci_smell,
            sci_drink,
            sci_funny,
            sci_scream,
        }
    }
}

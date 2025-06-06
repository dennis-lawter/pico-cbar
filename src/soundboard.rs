// use crate::pico::Pico;
// use crate::sound_library::SoundLibrary;
use crate::wav::Wav;

const MAX_PLAYLIST_SIZE: u8 = 6;

pub struct Playlist<'a> {
    index: u8,
    pub sounds: [Option<&'a [u8]>; MAX_PLAYLIST_SIZE as usize],
    length: u8,
}
impl<'a> Playlist<'a> {
    fn new() -> Self {
        let sounds = [const { None }; MAX_PLAYLIST_SIZE as usize];
        Self {
            index: 0u8,
            sounds,
            length: 0u8,
        }
    }

    fn insert(&mut self, sound: &'a [u8]) -> Result<(), ()> {
        if self.length >= MAX_PLAYLIST_SIZE {
            return Err(());
        }
        self.sounds[self.length as usize] = Some(sound.clone());
        self.length += 1;
        Ok(())
    }

    pub fn get_next(&mut self) -> &'a [u8] {
        if self.index >= self.length {
            self.index = 0;
        }
        let wav = self.sounds[self.index as usize].as_ref().unwrap();
        self.index += 1;
        wav
    }

    // pub fn get_rand(&mut self, index: usize) -> Wav<'static> {
    //     let index = index % self.length as usize;
    //     self.sounds[index as usize].clone().unwrap()
    // }
}

pub struct Soundboard<'a> {
    // pub snd_lib: SoundLibrary<'a>,
    pub cbar_miss: Playlist<'a>,
    pub cbar_metal: Playlist<'a>,
    pub cbar_body: Playlist<'a>,

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
    pub fn new() -> Self {
        // let snd_lib = SoundLibrary::default();

        let mut cbar_miss = Playlist::new();
        cbar_miss.insert(crate::sound_library::CBAR_MISS1).unwrap();
        let mut cbar_metal = Playlist::new();
        cbar_metal.insert(crate::sound_library::CBAR_HIT1).unwrap();
        cbar_metal.insert(crate::sound_library::CBAR_HIT2).unwrap();
        let mut cbar_body = Playlist::new();
        cbar_body
            .insert(crate::sound_library::CBAR_HITBOD1)
            .unwrap();
        cbar_body
            .insert(crate::sound_library::CBAR_HITBOD2)
            .unwrap();
        cbar_body
            .insert(crate::sound_library::CBAR_HITBOD3)
            .unwrap();

        let mut ba_smell = Playlist::new();
        ba_smell
            .insert(crate::sound_library::BA_SOMETHINGDIED)
            .unwrap();
        ba_smell
            .insert(crate::sound_library::BA_SOMETHINGSTINKY)
            .unwrap();
        ba_smell.insert(crate::sound_library::BA_STENCH).unwrap();
        let mut ba_drink = Playlist::new();
        ba_drink.insert(crate::sound_library::BA_COLDONE).unwrap();
        ba_drink.insert(crate::sound_library::BA_LATER).unwrap();
        ba_drink.insert(crate::sound_library::BA_LONGNITE).unwrap();
        ba_drink
            .insert(crate::sound_library::BA_BEERTOPSIDE)
            .unwrap();
        let mut ba_funny = Playlist::new();
        // ba_funny.insert(crate::sound_library::BA_BADAREA).unwrap();
        ba_funny
            .insert(crate::sound_library::BA_BADFEELING)
            .unwrap();
        // ba_funny.insert(crate::sound_library::BA_DOTOYOU).unwrap();
        ba_funny.insert(crate::sound_library::BA_SENDIT).unwrap();
        ba_funny.insert(crate::sound_library::BA_SURVIVE).unwrap();
        ba_funny
            .insert(crate::sound_library::BA_WHATSGOINGON)
            .unwrap();
        ba_funny
            .insert(crate::sound_library::BA_YOUTALKMUCH)
            .unwrap();
        let mut ba_scream = Playlist::new();
        ba_scream.insert(crate::sound_library::BA_DIE1).unwrap();
        ba_scream.insert(crate::sound_library::BA_DIE2).unwrap();
        ba_scream.insert(crate::sound_library::BA_DIE3).unwrap();
        // ba_scream.insert(crate::sound_library::BA_PAIN1).unwrap();
        ba_scream.insert(crate::sound_library::BA_PAIN2).unwrap();
        ba_scream.insert(crate::sound_library::BA_PAIN3).unwrap();

        let mut sci_smell = Playlist::new();
        sci_smell
            .insert(crate::sound_library::SCI_DOYOUSMELL)
            .unwrap();
        sci_smell
            .insert(crate::sound_library::SCI_ODORFROMYOU)
            .unwrap();
        sci_smell
            .insert(crate::sound_library::SCI_PECULIARODOR)
            .unwrap();
        sci_smell
            .insert(crate::sound_library::SCI_SOMETHINGFOUL)
            .unwrap();
        sci_smell.insert(crate::sound_library::SCI_STENCH).unwrap();
        let mut sci_drink = Playlist::new();
        sci_drink.insert(crate::sound_library::SCI_SEENCUP).unwrap();
        sci_drink
            .insert(crate::sound_library::SCI_HUNGRYYET)
            .unwrap();
        sci_drink
            .insert(crate::sound_library::SCI_NEEDSLEEP)
            .unwrap();
        let mut sci_funny = Playlist::new();
        // sci_funny.insert(crate::sound_library::SCI_FOOL).unwrap();
        // sci_funny.insert(crate::sound_library::SCI_DONTWANTDIE).unwrap();
        sci_funny
            .insert(crate::sound_library::SCI_DONUTEATER)
            .unwrap();
        sci_funny
            .insert(crate::sound_library::SCI_EXCELLENTTEAM)
            .unwrap();
        sci_funny
            .insert(crate::sound_library::SCI_HIDEGLASSES)
            .unwrap();
        sci_funny
            .insert(crate::sound_library::SCI_USELESSPHD)
            .unwrap();
        sci_funny.insert(crate::sound_library::SCI_WEARTIE).unwrap();
        let mut sci_scream = Playlist::new();
        sci_scream
            .insert(crate::sound_library::SCI_SCREAM1)
            .unwrap();
        sci_scream
            .insert(crate::sound_library::SCI_SCREAM2)
            .unwrap();
        sci_scream
            .insert(crate::sound_library::SCI_SCREAM3)
            .unwrap();
        sci_scream
            .insert(crate::sound_library::SCI_SCREAM4)
            .unwrap();
        sci_scream
            .insert(crate::sound_library::SCI_SCREAM5)
            .unwrap();
        // sci_scream.insert(crate::sound_library::SCI_SCREAM6).unwrap();
        // sci_scream.insert(crate::sound_library::SCI_SCREAM7).unwrap();

        Self {
            cbar_miss,
            cbar_metal,
            cbar_body,
            // snd_lib,
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

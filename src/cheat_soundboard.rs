// use crate::pico::Pico;
use crate::sound_library::SoundLibrary;
use crate::wav::Wav;

const MAX_PLAYLIST_SIZE: u8 = 6;

pub struct Playlist<'a> {
    index: u8,
    pub sounds: [Option<Wav<'a>>; MAX_PLAYLIST_SIZE as usize],
    length: u8,
}
impl Playlist<'static> {
    fn new() -> Self {
        let sounds = [const { None }; MAX_PLAYLIST_SIZE as usize];
        Self {
            index: 0u8,
            sounds,
            length: 0u8,
        }
    }

    fn insert(&mut self, sound: &Wav<'static>) -> Result<(), ()> {
        if self.length >= MAX_PLAYLIST_SIZE {
            return Err(());
        }
        self.sounds[self.length as usize] = Some(sound.clone());
        self.length += 1;
        Ok(())
    }

    pub fn get_next(&mut self) -> Wav<'static> {
        if self.index >= self.length || self.sounds[self.index as usize].is_none() {
            self.index = 0;
        }
        let wav = self.sounds[self.index as usize].clone().unwrap().clone();
        self.index += 1;
        wav
    }

    // pub fn get_rand(&mut self, index: usize) -> Wav<'static> {
    //     let index = index % self.length as usize;
    //     self.sounds[index as usize].clone().unwrap()
    // }
}

pub struct Soundboard<'a> {
    pub snd_lib: SoundLibrary<'a>,

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
        let snd_lib = SoundLibrary::default();

        let mut cbar_miss = Playlist::new();
        cbar_miss.insert(&snd_lib.cbar_miss1).unwrap();
        let mut cbar_metal = Playlist::new();
        cbar_metal.insert(&snd_lib.cbar_hit1).unwrap();
        cbar_metal.insert(&snd_lib.cbar_hit2).unwrap();
        let mut cbar_body = Playlist::new();
        cbar_body.insert(&snd_lib.cbar_hitbod1).unwrap();
        cbar_body.insert(&snd_lib.cbar_hitbod2).unwrap();
        cbar_body.insert(&snd_lib.cbar_hitbod3).unwrap();

        let mut ba_smell = Playlist::new();
        ba_smell.insert(&snd_lib.ba_somethingdied).unwrap();
        ba_smell.insert(&snd_lib.ba_somethingstinky).unwrap();
        ba_smell.insert(&snd_lib.ba_stench).unwrap();
        let mut ba_drink = Playlist::new();
        ba_drink.insert(&snd_lib.ba_coldone).unwrap();
        ba_drink.insert(&snd_lib.ba_later).unwrap();
        ba_drink.insert(&snd_lib.ba_longnite).unwrap();
        ba_drink.insert(&snd_lib.ba_beertopside).unwrap();
        let mut ba_funny = Playlist::new();
        // ba_funny.insert(&snd_lib.ba_badarea).unwrap();
        ba_funny.insert(&snd_lib.ba_badfeeling).unwrap();
        // ba_funny.insert(&snd_lib.ba_dotoyou).unwrap();
        ba_funny.insert(&snd_lib.ba_sendit).unwrap();
        ba_funny.insert(&snd_lib.ba_survive).unwrap();
        ba_funny.insert(&snd_lib.ba_whatsgoingon).unwrap();
        ba_funny.insert(&snd_lib.ba_youtalkmuch).unwrap();
        let mut ba_scream = Playlist::new();
        ba_scream.insert(&snd_lib.ba_die1).unwrap();
        ba_scream.insert(&snd_lib.ba_die2).unwrap();
        ba_scream.insert(&snd_lib.ba_die3).unwrap();
        // ba_scream.insert(&snd_lib.ba_pain1).unwrap();
        ba_scream.insert(&snd_lib.ba_pain2).unwrap();
        ba_scream.insert(&snd_lib.ba_pain3).unwrap();

        let mut sci_smell = Playlist::new();
        sci_smell.insert(&snd_lib.sci_doyousmell).unwrap();
        sci_smell.insert(&snd_lib.sci_odorfromyou).unwrap();
        sci_smell.insert(&snd_lib.sci_peculiarodor).unwrap();
        sci_smell.insert(&snd_lib.sci_somethingfoul).unwrap();
        sci_smell.insert(&snd_lib.sci_stench).unwrap();
        let mut sci_drink = Playlist::new();
        sci_drink.insert(&snd_lib.sci_seencup).unwrap();
        sci_drink.insert(&snd_lib.sci_hungryyet).unwrap();
        sci_drink.insert(&snd_lib.sci_needsleep).unwrap();
        let mut sci_funny = Playlist::new();
        // sci_funny.insert(&snd_lib.sci_fool).unwrap();
        // sci_funny.insert(&snd_lib.sci_dontwantdie).unwrap();
        sci_funny.insert(&snd_lib.sci_donuteater).unwrap();
        sci_funny.insert(&snd_lib.sci_excellentteam).unwrap();
        sci_funny.insert(&snd_lib.sci_hideglasses).unwrap();
        sci_funny.insert(&snd_lib.sci_uselessphd).unwrap();
        sci_funny.insert(&snd_lib.sci_weartie).unwrap();
        let mut sci_scream = Playlist::new();
        sci_scream.insert(&snd_lib.sci_scream1).unwrap();
        sci_scream.insert(&snd_lib.sci_scream2).unwrap();
        sci_scream.insert(&snd_lib.sci_scream3).unwrap();
        sci_scream.insert(&snd_lib.sci_scream4).unwrap();
        sci_scream.insert(&snd_lib.sci_scream5).unwrap();
        // sci_scream.insert(&snd_lib.sci_scream6).unwrap();
        // sci_scream.insert(&snd_lib.sci_scream7).unwrap();

        Self {
            cbar_miss,
            cbar_metal,
            cbar_body,
            snd_lib,
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

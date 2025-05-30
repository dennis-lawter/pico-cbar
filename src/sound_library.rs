use crate::wav::Wav;

macro_rules! include_wav {
    ($name:ident, $path:expr) => {
        const $name: &[u8] = include_bytes!($path);
    };
}

// CBAR sfx
include_wav!(CBAR_MISS1, "../sfx/weapons/cbar_miss1.wav");
include_wav!(CBAR_HIT1, "../sfx/weapons/cbar_hit1.wav");
include_wav!(CBAR_HIT2, "../sfx/weapons/cbar_hit2.wav");
include_wav!(CBAR_HITBOD1, "../sfx/weapons/cbar_hitbod1.wav");
include_wav!(CBAR_HITBOD2, "../sfx/weapons/cbar_hitbod2.wav");
include_wav!(CBAR_HITBOD3, "../sfx/weapons/cbar_hitbod3.wav");

// Button sound that plays when a wrong cheatcode is entered
include_wav!(BUTTON2, "../sfx/buttons/button2.wav");
// Barney "Heh, you're gonna wish you hadn't done that"
// Used when entering the cheat mode successfully
include_wav!(UWISH, "../sfx/barney/ba_uwish.wav");

// Cheat menu soundboard for Barney
// Barney's smell sfx
include_wav!(BA_SOMETHINGDIED, "../sfx/barney/somethingdied.wav");
include_wav!(BA_SOMETHINGSTINKY, "../sfx/barney/somethingstinky.wav");
include_wav!(BA_STENCH, "../sfx/barney/stench.wav");

// Barney's party sfx
include_wav!(BA_COLDONE, "../sfx/barney/coldone.wav");
include_wav!(BA_LATER, "../sfx/barney/ba_later.wav");
include_wav!(BA_LONGNITE, "../sfx/barney/c2a4_ba_longnite.wav");
include_wav!(BA_BEERTOPSIDE, "../sfx/barney/beertopside.wav");

// Barney's iconic voice lines
include_wav!(BA_BADAREA, "../sfx/barney/badarea.wav");
include_wav!(BA_BADFEELING, "../sfx/barney/badfeeling.wav");
// include_wav!(BA_DOTOYOU, "../sfx/barney/ba_dotoyou.wav");
include_wav!(BA_SENDIT, "../sfx/barney/c3a1_ba_3sat.wav");
include_wav!(BA_SURVIVE, "../sfx/barney/survive.wav");
include_wav!(BA_WHATSGOINGON, "../sfx/barney/whatsgoingon.wav");
include_wav!(BA_YOUTALKMUCH, "../sfx/barney/youtalkmuch.wav");

// Barney screaming
include_wav!(BA_DIE1, "../sfx/barney/ba_die1.wav");
include_wav!(BA_DIE2, "../sfx/barney/ba_die2.wav");
include_wav!(BA_DIE3, "../sfx/barney/ba_die3.wav");
// include_wav!(BA_PAIN1, "../sfx/barney/ba_pain1.wav");
include_wav!(BA_PAIN2, "../sfx/barney/ba_pain2.wav");
include_wav!(BA_PAIN3, "../sfx/barney/ba_pain3.wav");

// Cheat menu sfx for scientists
// Scientist smell sfx
include_wav!(SCI_DOYOUSMELL, "../sfx/scientist/doyousmell.wav");
include_wav!(SCI_ODORFROMYOU, "../sfx/scientist/odorfromyou.wav");
include_wav!(SCI_PECULIARODOR, "../sfx/scientist/peculiarodor.wav");
include_wav!(SCI_SOMETHINGFOUL, "../sfx/scientist/somethingfoul.wav");
include_wav!(SCI_STENCH, "../sfx/scientist/stench.wav");

// Scientist party sfx
include_wav!(SCI_SEENCUP, "../sfx/scientist/seencup.wav");
include_wav!(SCI_HUNGRYYET, "../sfx/scientist/hungryyet.wav");
include_wav!(SCI_NEEDSLEEP, "../sfx/scientist/needsleep.wav");

// Scientist iconic voice lines
// include_wav!(SCI_FOOL, "../sfx/scientist/c3a2_sci_fool.wav");
// include_wav!(SCI_DONTWANTDIE, "../sfx/scientist/dontwantdie.wav");
include_wav!(SCI_DONUTEATER, "../sfx/scientist/donuteater.wav");
include_wav!(SCI_EXCELLENTTEAM, "../sfx/scientist/excellentteam.wav");
include_wav!(SCI_HIDEGLASSES, "../sfx/scientist/hideglasses.wav");
include_wav!(SCI_USELESSPHD, "../sfx/scientist/uselessphd.wav");
include_wav!(SCI_WEARTIE, "../sfx/scientist/weartie.wav");

// Scientist screams
include_wav!(SCI_SCREAM1, "../sfx/scientist/scream1.wav");
include_wav!(SCI_SCREAM2, "../sfx/scientist/scream2.wav");
include_wav!(SCI_SCREAM3, "../sfx/scientist/scream3.wav");
include_wav!(SCI_SCREAM4, "../sfx/scientist/scream4.wav");
include_wav!(SCI_SCREAM5, "../sfx/scientist/scream5.wav");
// include_wav!(SCI_SCREAM6, "../sfx/scientist/scream6.wav");
// include_wav!(SCI_SCREAM7, "../sfx/scientist/scream7.wav");

pub struct SoundLibrary<'a> {
    pub cbar_hit1: Wav<'a>,
    pub cbar_hit2: Wav<'a>,

    pub cbar_hitbod1: Wav<'a>,
    pub cbar_hitbod2: Wav<'a>,
    pub cbar_hitbod3: Wav<'a>,

    pub cbar_miss1: Wav<'a>,

    pub wrong: Wav<'a>,
    pub uwish: Wav<'a>,

    pub ba_somethingdied: Wav<'a>,
    pub ba_somethingstinky: Wav<'a>,
    pub ba_stench: Wav<'a>,

    pub ba_coldone: Wav<'a>,
    pub ba_later: Wav<'a>,
    pub ba_longnite: Wav<'a>,
    pub ba_beertopside: Wav<'a>,

    // pub ba_badarea: Wav<'a>,
    pub ba_badfeeling: Wav<'a>,
    // pub ba_dotoyou: Wav<'a>,
    pub ba_sendit: Wav<'a>,
    pub ba_survive: Wav<'a>,
    pub ba_whatsgoingon: Wav<'a>,
    pub ba_youtalkmuch: Wav<'a>,

    pub ba_die1: Wav<'a>,
    pub ba_die2: Wav<'a>,
    pub ba_die3: Wav<'a>,
    // pub ba_pain1: Wav<'a>,
    pub ba_pain2: Wav<'a>,
    pub ba_pain3: Wav<'a>,

    pub sci_doyousmell: Wav<'a>,
    pub sci_odorfromyou: Wav<'a>,
    pub sci_peculiarodor: Wav<'a>,
    pub sci_somethingfoul: Wav<'a>,
    pub sci_stench: Wav<'a>,

    pub sci_seencup: Wav<'a>,
    pub sci_hungryyet: Wav<'a>,
    pub sci_needsleep: Wav<'a>,

    // pub sci_fool: Wav<'a>,
    // pub sci_dontwantdie: Wav<'a>,
    pub sci_donuteater: Wav<'a>,
    pub sci_excellentteam: Wav<'a>,
    pub sci_hideglasses: Wav<'a>,
    pub sci_uselessphd: Wav<'a>,
    pub sci_weartie: Wav<'a>,

    pub sci_scream1: Wav<'a>,
    pub sci_scream2: Wav<'a>,
    pub sci_scream3: Wav<'a>,
    pub sci_scream4: Wav<'a>,
    pub sci_scream5: Wav<'a>,
    // pub sci_scream6: Wav<'a>,
    // pub sci_scream7: Wav<'a>,
}
impl Default for SoundLibrary<'static> {
    fn default() -> Self {
        Self {
            cbar_hit1: Wav::new(CBAR_HIT1),
            cbar_hit2: Wav::new(CBAR_HIT2),
            cbar_hitbod1: Wav::new(CBAR_HITBOD1),
            cbar_hitbod2: Wav::new(CBAR_HITBOD2),
            cbar_hitbod3: Wav::new(CBAR_HITBOD3),
            cbar_miss1: Wav::new(CBAR_MISS1),
            wrong: Wav::new(BUTTON2),
            uwish: Wav::new(UWISH),
            ba_somethingdied: Wav::new(BA_SOMETHINGDIED),
            ba_somethingstinky: Wav::new(BA_SOMETHINGSTINKY),
            ba_stench: Wav::new(BA_STENCH),
            ba_coldone: Wav::new(BA_COLDONE),
            ba_later: Wav::new(BA_LATER),
            ba_longnite: Wav::new(BA_LONGNITE),
            ba_beertopside: Wav::new(BA_BEERTOPSIDE),
            // ba_badarea: Wav::new(BA_BADAREA),
            ba_badfeeling: Wav::new(BA_BADFEELING),
            // ba_dotoyou: Wav::new(BA_DOTOYOU),
            ba_sendit: Wav::new(BA_SENDIT),
            ba_survive: Wav::new(BA_SURVIVE),
            ba_whatsgoingon: Wav::new(BA_WHATSGOINGON),
            ba_youtalkmuch: Wav::new(BA_YOUTALKMUCH),
            ba_die1: Wav::new(BA_DIE1),
            ba_die2: Wav::new(BA_DIE2),
            ba_die3: Wav::new(BA_DIE3),
            // ba_pain1: Wav::new(BA_PAIN1),
            ba_pain2: Wav::new(BA_PAIN2),
            ba_pain3: Wav::new(BA_PAIN3),
            sci_doyousmell: Wav::new(SCI_DOYOUSMELL),
            sci_odorfromyou: Wav::new(SCI_ODORFROMYOU),
            sci_peculiarodor: Wav::new(SCI_PECULIARODOR),
            sci_somethingfoul: Wav::new(SCI_SOMETHINGFOUL),
            sci_stench: Wav::new(SCI_STENCH),
            sci_seencup: Wav::new(SCI_SEENCUP),
            sci_hungryyet: Wav::new(SCI_HUNGRYYET),
            sci_needsleep: Wav::new(SCI_NEEDSLEEP),
            // sci_fool: Wav::new(SCI_FOOL),
            // sci_dontwantdie: Wav::new(SCI_DONTWANTDIE),
            sci_donuteater: Wav::new(SCI_DONUTEATER),
            sci_excellentteam: Wav::new(SCI_EXCELLENTTEAM),
            sci_hideglasses: Wav::new(SCI_HIDEGLASSES),
            sci_uselessphd: Wav::new(SCI_USELESSPHD),
            sci_weartie: Wav::new(SCI_WEARTIE),
            sci_scream1: Wav::new(SCI_SCREAM1),
            sci_scream2: Wav::new(SCI_SCREAM2),
            sci_scream3: Wav::new(SCI_SCREAM3),
            sci_scream4: Wav::new(SCI_SCREAM4),
            sci_scream5: Wav::new(SCI_SCREAM5),
            // sci_scream6: Wav::new(SCI_SCREAM6),
            // sci_scream7: Wav::new(SCI_SCREAM7),
        }
    }
}

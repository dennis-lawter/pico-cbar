macro_rules! include_wav {
    ($name:ident, $path:expr) => {
        pub const $name: &[u8] = include_bytes!($path);
    };
}

// Below is every wav file uploaded to the pico
// Due to the 2MB ROM limit some sounds were cut

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

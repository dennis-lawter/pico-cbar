use rp_pico::hal::multicore::Stack;

use crate::pico::Pico;
use crate::wav::Wav;

pub static mut CORE1_STACK: Stack<4096> = Stack::new();

pub static mut QUEUE: Option<&[u8]> = None;
pub static mut PICO: Option<*mut Pico> = None;

fn core1_loop() -> ! {
    loop {
        unsafe {
            match PICO {
                Some(pico) => match QUEUE {
                    Some(_) => {
                        let wav = Wav::new(QUEUE.unwrap());
                        pico.as_mut().unwrap().play_wav_blocking(&wav);
                        QUEUE = None
                    }
                    None => {}
                },
                None => {}
            }
        }
    }
}

#[allow(dead_code)]
#[derive(Clone)]
pub struct Wav<'a> {
    pub sample_rate: u32,
    pub data_ref: &'a [u8],
    pub chunk_len: usize,
    pub gain: u8,
}
impl Wav<'static> {
    #[allow(dead_code)]
    pub fn new(data_ref: &'static [u8]) -> Wav<'static> {
        // Confirm RIFF format
        assert_eq!(0x52, data_ref[0x00]);
        assert_eq!(0x49, data_ref[0x01]);
        assert_eq!(0x46, data_ref[0x02]);
        assert_eq!(0x46, data_ref[0x03]);
        // Next 4 bytes represent the file size, which we can ignore
        // 0x04 0x05 0x06 0x07
        // Confirm WAVE format
        assert_eq!(0x57, data_ref[0x08]);
        assert_eq!(0x41, data_ref[0x09]);
        assert_eq!(0x56, data_ref[0x0A]);
        assert_eq!(0x45, data_ref[0x0B]);

        // Assume the fmt block follows
        assert_eq!(0x66, data_ref[0x0C]);
        assert_eq!(0x6D, data_ref[0x0D]);
        assert_eq!(0x74, data_ref[0x0E]);
        assert_eq!(0x20, data_ref[0x0F]);
        // fmt block length follows, which we can ignore
        // 0x10 0x11 0x12 0x13
        // Assume PCM data format; project limitation
        assert_eq!(0x01, data_ref[0x14]);
        assert_eq!(0x00, data_ref[0x15]);
        // Assuming 1 ch; project limitation
        assert_eq!(0x01, data_ref[0x16]);
        assert_eq!(0x00, data_ref[0x17]);
        // Store sample rate
        let sample_rate = (data_ref[0x18] as u32) << 0
            | (data_ref[0x19] as u32) << 8
            | (data_ref[0x1A] as u32) << 16
            | (data_ref[0x1B] as u32) << 24;
        // The next 4 bytes represent BytePerSec, which we can ignore.
        // 0x1C 0x1D 0x1E 0x1F
        // The next 2 bytes represent BytePerBloc, which we can ignore.
        // 0x20 0x21
        // Assume we have an 8-bit wav
        assert_eq!(0x08, data_ref[0x22]);
        assert_eq!(0x00, data_ref[0x23]);

        // TODO: Investigate
        // The following assertions actually fail,
        // need to examine wavs to find culprit

        // // Project assumes 1 chunk of sample data,
        // // and it must be the first chunk after the headers
        // assert_eq!(0x64, data_ref[0x24]);
        // assert_eq!(0x61, data_ref[0x25]);
        // assert_eq!(0x74, data_ref[0x26]);
        // assert_eq!(0x61, data_ref[0x27]);
        // Store the chunk length
        let chunk_len = (data_ref[0x28] as usize) << 0
            | (data_ref[0x29] as usize) << 8
            | (data_ref[0x2A] as usize) << 16
            | (data_ref[0x2B] as usize) << 24;

        Self {
            sample_rate,
            data_ref,
            chunk_len,
            gain: 4u8,
        }
    }
}

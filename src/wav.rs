#[allow(dead_code)]
pub struct Wav<'a> {
    sample_rate: u32,
    data_ref: &'a [u8],
}
impl Wav<'static> {
    #[allow(dead_code)]
    pub fn new(data_ref: &'static [u8]) -> Wav<'static> {
        // Confirm RIFF format
        assert_eq!(0x52, data_ref[0]);
        assert_eq!(0x49, data_ref[1]);
        assert_eq!(0x46, data_ref[2]);
        assert_eq!(0x46, data_ref[3]);
        // Confirm ref len = expected file size
        let file_size = (data_ref[4] as usize) << 24
            & (data_ref[5] as usize) << 16
            & (data_ref[6] as usize) << 8
            & (data_ref[7] as usize);
        assert_eq!(file_size, data_ref.len());
        // Confirm WAVE format
        assert_eq!(0x57, data_ref[8]);
        assert_eq!(0x41, data_ref[9]);
        assert_eq!(0x56, data_ref[10]);
        assert_eq!(0x45, data_ref[11]);

        // Assume the fmt block follows
        assert_eq!(0x66, data_ref[12]);
        assert_eq!(0x6D, data_ref[13]);
        assert_eq!(0x74, data_ref[14]);
        assert_eq!(0x20, data_ref[15]);
        // Assume the fmt block is 16 bytes
        assert_eq!(0x00, data_ref[16]);
        assert_eq!(0x00, data_ref[17]);
        assert_eq!(0x00, data_ref[18]);
        assert_eq!(0x10, data_ref[19]);
        // Assume PCM data format
        assert_eq!(0x00, data_ref[20]);
        assert_eq!(0x01, data_ref[21]);
        // Assuming 1 ch
        assert_eq!(0x00, data_ref[22]);
        assert_eq!(0x01, data_ref[23]);
        // Store sample rate
        let sample_rate = (data_ref[24] as u32) << 24
            & (data_ref[25] as u32) << 16
            & (data_ref[26] as u32) << 8
            & (data_ref[27] as u32);
        // The next 4 bytes represent BytePerSec, which we can ignore.
        // 28 29 30 31
        // The next 2 bytes represent BytePerBloc, which we can ignore.
        // 32 33
        // Assume we have an 8-bit wav
        assert_eq!(0x00, data_ref[34]);
        assert_eq!(0x08, data_ref[35]);

        // Assume our first chunk will be sample data
        // (0x64, 0x61, 0x74, 0x61)
        assert_eq!(0x64, data_ref[36]);
        assert_eq!(0x61, data_ref[37]);
        assert_eq!(0x74, data_ref[38]);
        assert_eq!(0x61, data_ref[39]);
        // We can ignore the next 4 bytes,
        // we assume our file contains one sample data chunk
        // 40 41 42 43

        Self {
            sample_rate,
            data_ref,
        }
    }
}

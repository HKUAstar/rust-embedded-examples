//! DR16's 18-byte DBUS frame. USART3 receives it through the board's inverter at 100000 baud 8E2.

/// Decoded operator inputs; channels are centered at zero, nominal range -660 through 660.
#[derive(Clone, Copy, Debug)]
pub struct Remote {
    pub channels: [i16; 5],
    /// Raw switches: 1 up, 3 middle, 2 down. Index zero is the right switch.
    pub switches: [u8; 2],
    pub mouse: [i16; 3],
    pub buttons: [bool; 2],
    pub keys: u16,
}

impl Remote {
    /// Rejects wrong lengths, invalid switches/buttons and out-of-range channels.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let b: &[u8; 18] = bytes.try_into().ok()?;
        let word = |i| u16::from(b[i]);
        let channels = [
            (word(0) | word(1) << 8) & 0x7ff,
            (word(1) >> 3 | word(2) << 5) & 0x7ff,
            (word(2) >> 6 | word(3) << 2 | word(4) << 10) & 0x7ff,
            (word(4) >> 1 | word(5) << 7) & 0x7ff,
            u16::from_le_bytes([b[16], b[17]]),
        ]
        .map(|value| i32::from(value) - 1024);
        let switches = [(b[5] >> 4) & 3, b[5] >> 6];
        if channels.iter().any(|value| !(-660..=660).contains(value))
            || switches.iter().any(|value| !(1..=3).contains(value))
            || b[12] > 1
            || b[13] > 1
        {
            return None;
        }
        Some(Self {
            channels: channels.map(|value| value as i16),
            switches,
            mouse: core::array::from_fn(|i| i16::from_le_bytes([b[6 + 2 * i], b[7 + 2 * i]])),
            buttons: [b[12] != 0, b[13] != 0],
            keys: u16::from_le_bytes([b[14], b[15]]),
        })
    }

    /// Normalized stick with a 10-count deadband.
    ///
    /// # Panics
    ///
    /// Panics if `channel` is outside 0..5.
    pub fn stick(&self, channel: usize) -> f32 {
        let value = self.channels[channel];
        if value.abs() < 10 {
            0.0
        } else {
            f32::from(value) / 660.0
        }
    }
}

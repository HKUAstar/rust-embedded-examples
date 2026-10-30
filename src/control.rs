//! Small control calculations used unchanged by the chassis, gimbal and combined lessons.

use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant};

/// Proportional/integral controller with bounded output and integral state.
#[derive(Debug)]
pub struct Pi {
    kp: f32,
    ki: f32,
    integral: f32,
    limit: f32,
}
impl Pi {
    /// Gains are nonnegative; `ki` integrates error over elapsed seconds.
    ///
    /// # Panics
    ///
    /// Panics if a gain is negative or non-finite, or the output limit is not finite and positive.
    pub fn new(kp: f32, ki: f32, limit: f32) -> Self {
        assert!(
            kp.is_finite() && kp >= 0.0,
            "invalid proportional gain: {kp}"
        );
        assert!(ki.is_finite() && ki >= 0.0, "invalid integral gain: {ki}");
        assert!(
            limit.is_finite() && limit > 0.0,
            "invalid output limit: {limit}"
        );
        Self {
            kp,
            ki,
            integral: 0.0,
            limit,
        }
    }
    /// Clears accumulated error, for example when input expires.
    pub fn reset(&mut self) {
        self.integral = 0.0;
    }
    /// Returns bounded output; invalid error or a nonpositive/non-finite time step resets to zero.
    /// Callers reject time gaps that are too long for their control loop before calling this.
    pub fn update(&mut self, error: f32, dt_s: f32) -> f32 {
        if !error.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            self.reset();
            return 0.0;
        }
        let candidate = (self.integral + self.ki * error * dt_s).clamp(-self.limit, self.limit);
        let output = self.kp * error + candidate;
        if output.abs() < self.limit || output.signum() != error.signum() {
            self.integral = candidate;
        }
        (self.kp * error + self.integral).clamp(-self.limit, self.limit)
    }
}

/// DJI feedback, using the CAN envelope's reception time to reject queued stale frames.
#[derive(Clone, Copy, Debug)]
pub struct Motor {
    pub angle_rad: f32,
    pub velocity_rad_s: f32,
    pub current_raw: i16,
    pub temperature_c: u8,
    pub received_at: Instant,
}
impl Motor {
    /// Decoder for 8-byte M3508/M2006/GM6020 feedback; velocity is at the motor rotor.
    pub fn decode(frame: &Frame, received_at: Instant) -> Option<Self> {
        if frame.header().rtr() || frame.header().len() != 8 {
            return None;
        }
        let b = frame.data();
        let encoder = u16::from_be_bytes([b[0], b[1]]);
        if encoder >= 8192 {
            return None;
        }
        Some(Self {
            angle_rad: f32::from(encoder) * core::f32::consts::TAU / 8192.0,
            velocity_rad_s: f32::from(i16::from_be_bytes([b[2], b[3]])) * core::f32::consts::TAU
                / 60.0,
            current_raw: i16::from_be_bytes([b[4], b[5]]),
            temperature_c: b[6],
            received_at,
        })
    }
    /// Feedback expires 20 ms after reception.
    pub fn fresh(&self, now: Instant) -> bool {
        now.saturating_duration_since(self.received_at) < Duration::from_millis(20)
    }
}

/// Four signed, big-endian raw command slots; their units depend on the motor controller.
///
/// # Panics
///
/// Panics if `id` is not an 11-bit standard CAN identifier.
pub fn command(id: u16, commands_raw: [i16; 4]) -> Frame {
    let mut bytes = [0; 8];
    for (value, slot) in commands_raw
        .into_iter()
        .zip(bytes.as_chunks_mut::<2>().0.iter_mut())
    {
        slot.copy_from_slice(&value.to_be_bytes());
    }
    Frame::new_standard(id, &bytes).expect("DJI command ID is standard and length is eight")
}

/// Principal angular error, in [-pi, pi].
pub fn angle_error(target_rad: f32, measured_rad: f32) -> f32 {
    let delta = target_rad - measured_rad;
    libm::atan2f(libm::sinf(delta), libm::cosf(delta))
}

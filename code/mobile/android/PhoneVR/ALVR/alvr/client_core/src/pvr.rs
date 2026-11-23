// Phone PCVR additions to the ALVR client core.
//
// Everything here is client-side only: the wire protocol is unchanged, so this client still talks
// to a stock ALVR v20.8.0 streamer. The phone app sets the options at any time through
// alvr_pvr_set_options() and reads the counters through alvr_pvr_get_stats().

use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU64, Ordering::Relaxed};
use std::time::Instant;

// ---- Options ----

// Present only the newest decoded frame and drop older queued ones. Stock ALVR presents frames
// oldest-first from a queue of up to ~2 frames, which adds up to two refreshes of latency.
static LATEST_FRAME_ONLY: AtomicBool = AtomicBool::new(false);
// Overrides the streamer's video.max_buffering_frames (stored as f32 bits; negative = use the
// streamer's value).
static MAX_BUFFERING_FRAMES_BITS: AtomicU32 = AtomicU32::new(0xBF80_0000); // -1.0
// How long the network thread may wait for a free decoder input buffer before dropping the frame.
// Stock ALVR waits 0 ms; every drop corrupts the picture until the next keyframe.
static DECODER_INPUT_WAIT_MS: AtomicU32 = AtomicU32::new(0);
// -1 = streamer's connection.avoid_video_glitching, 0 = show corrupted frames, 1 = hide them
// (the app keeps re-warping the last good frame until a keyframe arrives).
static HIDE_CORRUPTED: AtomicI32 = AtomicI32::new(-1);
// Adds MediaFormat "low-latency" = 1 (Android 11+) when the decoder is created.
static LOW_LATENCY_DECODER: AtomicBool = AtomicBool::new(true);

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AlvrPvrOptions {
    pub latest_frame_only: bool,
    // Negative: use the streamer's value.
    pub max_buffering_frames: f32,
    pub decoder_input_wait_ms: u32,
    // -1 streamer, 0 show, 1 hide
    pub hide_corrupted_frames: i32,
    pub low_latency_decoder: bool,
}

pub fn set_options(o: &AlvrPvrOptions) {
    LATEST_FRAME_ONLY.store(o.latest_frame_only, Relaxed);
    MAX_BUFFERING_FRAMES_BITS.store(o.max_buffering_frames.to_bits(), Relaxed);
    DECODER_INPUT_WAIT_MS.store(o.decoder_input_wait_ms.min(50), Relaxed);
    HIDE_CORRUPTED.store(o.hide_corrupted_frames.clamp(-1, 1), Relaxed);
    LOW_LATENCY_DECODER.store(o.low_latency_decoder, Relaxed);
}

pub fn latest_frame_only() -> bool {
    LATEST_FRAME_ONLY.load(Relaxed)
}

pub fn max_buffering_frames(server_value: f32) -> f32 {
    let v = f32::from_bits(MAX_BUFFERING_FRAMES_BITS.load(Relaxed));
    if v < 0.0 {
        server_value
    } else {
        v
    }
}

// The app's buffer override as a whole number of spare frames (None = streamer's setting and
// ALVR's running-average logic).
pub fn spare_frames_override() -> Option<usize> {
    let v = f32::from_bits(MAX_BUFFERING_FRAMES_BITS.load(Relaxed));
    (v >= 0.0).then(|| v.floor() as usize)
}

pub fn decoder_input_wait_ms() -> u32 {
    DECODER_INPUT_WAIT_MS.load(Relaxed)
}

pub fn hide_corrupted(server_value: bool) -> bool {
    match HIDE_CORRUPTED.load(Relaxed) {
        0 => false,
        1 => true,
        _ => server_value,
    }
}

pub fn low_latency_decoder() -> bool {
    LOW_LATENCY_DECODER.load(Relaxed)
}

// ---- Statistics ----
// Monotonic counters (the app diffs them per second) plus a few "since last read" maxima.

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct AlvrPvrStats {
    // Video frames fully received from the network.
    pub frames_received: u64,
    // Frames the network layer reported as having lost packets (UDP only).
    pub frames_with_packet_loss: u64,
    // Frames pushed into the decoder.
    pub frames_decoded_in: u64,
    // Frames that came out of the decoder.
    pub frames_decoded_out: u64,
    // Frames handed to the app by alvr_get_frame.
    pub frames_presented: u64,
    // Decoded frames thrown away because a newer one was ready (latest-frame mode / buffering cap).
    pub frames_dropped_stale: u64,
    // Frames dropped because the decoder had no free input buffer in time.
    pub frames_dropped_decoder_full: u64,
    // Frames not decoded because the picture was corrupted and we wait for a keyframe.
    pub frames_dropped_corrupt: u64,
    // Keyframe (IDR) requests sent to the streamer.
    pub idr_requests: u64,
    // Keyframes received.
    pub idr_received: u64,
    // Sum / count of the time between consecutive received frames (us).
    pub arrival_gap_sum_us: u64,
    pub arrival_gap_count: u64,
    // Largest gap between received frames since the previous alvr_pvr_get_stats call (us).
    pub arrival_gap_max_us: u64,
    // Sum of receive -> decoder output time (us), over frames_decoded_out.
    pub decode_sum_us: u64,
    // Sum of decoder output -> presented time (us), over frames_presented.
    pub queue_sum_us: u64,
    // Largest decoded-frame queue length seen since the previous read.
    pub queue_len_max: u64,
    // Sum of decoder input wait (us), over frames_decoded_in.
    pub input_wait_sum_us: u64,
    // Frames whose exact tracking sample was gone, so the nearest one was used (backlog sign).
    pub pose_fallbacks: u64,
}

macro_rules! counters {
    ($($name:ident),*) => {
        $(static $name: AtomicU64 = AtomicU64::new(0);)*
    };
}

counters!(
    FRAMES_RECEIVED,
    FRAMES_WITH_PACKET_LOSS,
    FRAMES_DECODED_IN,
    FRAMES_DECODED_OUT,
    FRAMES_PRESENTED,
    FRAMES_DROPPED_STALE,
    FRAMES_DROPPED_DECODER_FULL,
    FRAMES_DROPPED_CORRUPT,
    IDR_REQUESTS,
    IDR_RECEIVED,
    ARRIVAL_GAP_SUM_US,
    ARRIVAL_GAP_COUNT,
    ARRIVAL_GAP_MAX_US,
    DECODE_SUM_US,
    QUEUE_SUM_US,
    QUEUE_LEN_MAX,
    INPUT_WAIT_SUM_US,
    POSE_FALLBACKS,
    LAST_ARRIVAL_US
);

static EPOCH: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();

// Microseconds since the first call (monotonic).
pub fn now_us() -> u64 {
    EPOCH.get_or_init(Instant::now).elapsed().as_micros() as u64
}

fn add(c: &AtomicU64, v: u64) {
    c.fetch_add(v, Relaxed);
}

fn max(c: &AtomicU64, v: u64) {
    c.fetch_max(v, Relaxed);
}

pub fn on_frame_received(is_idr: bool, had_packet_loss: bool) {
    add(&FRAMES_RECEIVED, 1);
    if is_idr {
        add(&IDR_RECEIVED, 1);
    }
    if had_packet_loss {
        add(&FRAMES_WITH_PACKET_LOSS, 1);
    }
    let now = now_us();
    let last = LAST_ARRIVAL_US.swap(now, Relaxed);
    if last != 0 && now > last {
        let gap = now - last;
        // Ignore gaps across stream restarts.
        if gap < 5_000_000 {
            add(&ARRIVAL_GAP_SUM_US, gap);
            add(&ARRIVAL_GAP_COUNT, 1);
            max(&ARRIVAL_GAP_MAX_US, gap);
        }
    }
}

// Receive times of recent frames, keyed by the frame timestamp, to measure decode time.
static RECEIVE_TIMES: std::sync::Mutex<[(u64, u64); 32]> = std::sync::Mutex::new([(0, 0); 32]);

pub fn note_receive_time(timestamp_ns: u64) {
    if let Ok(mut ring) = RECEIVE_TIMES.lock() {
        let i = (timestamp_ns / 1000) as usize % ring.len();
        ring[i] = (timestamp_ns, now_us());
    }
}

// Microseconds since the frame with this timestamp was received, if still known.
pub fn since_receive_us(timestamp_ns: u64) -> Option<u64> {
    let ring = RECEIVE_TIMES.lock().ok()?;
    let (ts, t) = ring[(timestamp_ns / 1000) as usize % ring.len()];
    (ts == timestamp_ns).then(|| now_us().saturating_sub(t))
}

pub fn on_decoder_input(wait_us: u64) {
    add(&FRAMES_DECODED_IN, 1);
    add(&INPUT_WAIT_SUM_US, wait_us);
}

pub fn on_decoder_output(decode_us: u64) {
    add(&FRAMES_DECODED_OUT, 1);
    add(&DECODE_SUM_US, decode_us);
}

pub fn on_presented(queue_us: u64, queue_len: usize) {
    add(&FRAMES_PRESENTED, 1);
    add(&QUEUE_SUM_US, queue_us);
    max(&QUEUE_LEN_MAX, queue_len as u64);
}

pub fn on_dropped_stale(n: u64) {
    add(&FRAMES_DROPPED_STALE, n);
}

pub fn on_dropped_decoder_full() {
    add(&FRAMES_DROPPED_DECODER_FULL, 1);
}

pub fn on_dropped_corrupt() {
    add(&FRAMES_DROPPED_CORRUPT, 1);
}

pub fn on_pose_fallback() {
    add(&POSE_FALLBACKS, 1);
}

pub fn on_idr_request() {
    add(&IDR_REQUESTS, 1);
}

pub fn get_stats() -> AlvrPvrStats {
    AlvrPvrStats {
        frames_received: FRAMES_RECEIVED.load(Relaxed),
        frames_with_packet_loss: FRAMES_WITH_PACKET_LOSS.load(Relaxed),
        frames_decoded_in: FRAMES_DECODED_IN.load(Relaxed),
        frames_decoded_out: FRAMES_DECODED_OUT.load(Relaxed),
        frames_presented: FRAMES_PRESENTED.load(Relaxed),
        frames_dropped_stale: FRAMES_DROPPED_STALE.load(Relaxed),
        frames_dropped_decoder_full: FRAMES_DROPPED_DECODER_FULL.load(Relaxed),
        frames_dropped_corrupt: FRAMES_DROPPED_CORRUPT.load(Relaxed),
        idr_requests: IDR_REQUESTS.load(Relaxed),
        idr_received: IDR_RECEIVED.load(Relaxed),
        arrival_gap_sum_us: ARRIVAL_GAP_SUM_US.load(Relaxed),
        arrival_gap_count: ARRIVAL_GAP_COUNT.load(Relaxed),
        arrival_gap_max_us: ARRIVAL_GAP_MAX_US.swap(0, Relaxed),
        decode_sum_us: DECODE_SUM_US.load(Relaxed),
        queue_sum_us: QUEUE_SUM_US.load(Relaxed),
        queue_len_max: QUEUE_LEN_MAX.swap(0, Relaxed),
        input_wait_sum_us: INPUT_WAIT_SUM_US.load(Relaxed),
        pose_fallbacks: POSE_FALLBACKS.load(Relaxed),
    }
}

#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>
#include <cmath>

namespace AudioMetering {

/**
 * @brief Fast integer square root for single-core RISC-V (ESP32-C6) integer ALU.
 * Eliminates floating-point emulation in time-critical audio tasks.
 */
static inline uint32_t isqrt32(uint32_t val) {
    uint32_t res = 0;
    uint32_t bit = 1u << 30;
    while (bit > val) bit >>= 2;
    while (bit != 0) {
        if (val >= res + bit) {
            val -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/**
 * @brief Thread-safe Single-Producer, Single-Consumer (SPSC) lock-free ring buffer.
 * Sized statically at compile time (typically 100 frames = 1.0 second at 100 Hz).
 */
template <typename T, size_t Capacity>
class SpscRingBuffer {
public:
    static_assert(Capacity > 0, "Capacity must be greater than zero");

    inline void push(T item) {
        size_t head = m_head.load(std::memory_order_relaxed);
        m_buffer[head] = item;
        m_head.store((head + 1) % Capacity, std::memory_order_release);
        m_total_pushed.fetch_add(1, std::memory_order_release);
    }

    size_t getRecent(T* out_array, size_t max_items) const {
        size_t total = m_total_pushed.load(std::memory_order_acquire);
        size_t head = m_head.load(std::memory_order_acquire);
        size_t available = (total < Capacity) ? total : Capacity;
        size_t n = (max_items < available) ? max_items : available;
        for (size_t i = 0; i < n; ++i) {
            size_t idx = (head + Capacity - 1 - i) % Capacity;
            out_array[i] = m_buffer[idx];
        }
        return n;
    }

    inline void clear() {
        m_head.store(0, std::memory_order_release);
        m_total_pushed.store(0, std::memory_order_release);
    }

    inline size_t size() const {
        size_t total = m_total_pushed.load(std::memory_order_acquire);
        return (total < Capacity) ? total : Capacity;
    }

    inline size_t capacity() const {
        return Capacity;
    }

private:
    volatile T m_buffer[Capacity] = {0};
    std::atomic<size_t> m_head{0};
    std::atomic<size_t> m_total_pushed{0};
};

/**
 * @brief Complete Audio Signal Meter with dual SPSC ring buffers for Peak & RMS statistics.
 * Fixed-point single-pass producer functions optimized for real-time task execution.
 */
class AudioSignalMeter {
public:
    static constexpr size_t DEFAULT_HISTORY_FRAMES = 100; // 1.0s @ 100 fps

    AudioSignalMeter() = default;
    ~AudioSignalMeter() = default;

    /* =========================================================================
     * PRODUCER FUNCTIONS (INLINE - Zero Task Overhead on ESP32-C6 Integer ALU)
     * ========================================================================= */

    /**
     * @brief Computes Peak and RMS in a single pass over 16-bit PCM samples and pushes to ring buffers.
     * Peak: peak-to-peak / 2 = (max() - min()) / 2.
     * RMS:  integer isqrt(sum(sample^2) / N).
     * @param pcm Array of signed 16-bit PCM samples.
     * @param num_samples Number of samples in the frame (e.g. 480 for 10ms @ 48kHz).
     */
    inline void pushFramePcm(const int16_t* pcm, size_t num_samples, int stride = 1) {
        if (__builtin_expect(pcm == nullptr || num_samples == 0, 0)) {
            pushSilence();
            return;
        }

        int16_t min_s = pcm[0];
        int16_t max_s = pcm[0];
        int64_t sum_sq = 0;

        for (size_t i = 0; i < num_samples; ++i) {
            int16_t s = pcm[i * stride];
            if (s < min_s) min_s = s;
            if (s > max_s) max_s = s;
            sum_sq += static_cast<int32_t>(s) * static_cast<int32_t>(s);
        }

        int32_t pk2pk = static_cast<int32_t>(max_s) - static_cast<int32_t>(min_s);
        int16_t frame_peak = static_cast<int16_t>(pk2pk / 2);
        if (frame_peak < 0) frame_peak = 32767;

        uint32_t mean_sq = static_cast<uint32_t>(sum_sq / num_samples);
        int16_t frame_rms = static_cast<int16_t>(isqrt32(mean_sq));

        m_peak_ring_buf.push(frame_peak);
        m_rms_ring_buf.push(frame_rms);
        m_total_frames.fetch_add(1, std::memory_order_relaxed);
    }

    inline void pushPeak(int16_t peak) {
        m_peak_ring_buf.push(peak);
    }

    inline void pushRMS(int16_t rms) {
        m_rms_ring_buf.push(rms);
    }

    inline void pushSilence() {
        m_peak_ring_buf.push(0);
        m_rms_ring_buf.push(0);
    }

    inline void reset() {
        m_peak_ring_buf.clear();
        m_rms_ring_buf.clear();
        m_total_frames.store(0, std::memory_order_relaxed);
    }

    /* =========================================================================
     * CONSUMER FUNCTIONS (Called from lower-priority tasks / Heartbeat / Web API)
     * ========================================================================= */

    int16_t getAudioFramePeak_int16(unsigned int frameCount = DEFAULT_HISTORY_FRAMES) const;
    float getAudioFramePeak_dBFS(unsigned int frameCount = DEFAULT_HISTORY_FRAMES) const;
    float getAudioFramePeak_pct(unsigned int frameCount = DEFAULT_HISTORY_FRAMES) const;

    int16_t getAudioFrameRMS_int16(unsigned int frameCount = DEFAULT_HISTORY_FRAMES) const;
    float getAudioFrameRMS_dBFS(unsigned int frameCount = DEFAULT_HISTORY_FRAMES) const;
    float getAudioFrameRMS_pct(unsigned int frameCount = DEFAULT_HISTORY_FRAMES) const;

    uint32_t getTotalProcessedFrames() const {
        return m_total_frames.load(std::memory_order_relaxed);
    }

private:
    SpscRingBuffer<int16_t, DEFAULT_HISTORY_FRAMES> m_peak_ring_buf;
    SpscRingBuffer<int16_t, DEFAULT_HISTORY_FRAMES> m_rms_ring_buf;
    std::atomic<uint32_t> m_total_frames{0};
};

} // namespace AudioMetering

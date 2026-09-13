#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <atomic>
#include <algorithm>
#include <type_traits>

namespace stats {

/**
 * @brief Zero-allocation, thread-safe Single-Producer ring buffer with statistical analysis.
 * 
 * Sized statically at compile time. Zero dynamic memory allocation (no heap/malloc).
 * Lock-free, non-blocking push() optimized for high-priority FreeRTOS tasks and ISRs.
 * 
 * @tparam T Word type (e.g. int16_t, uint16_t, int32_t, uint32_t, float, double)
 * @tparam Capacity Number of elements in the ring buffer (> 0)
 */
template <typename T, size_t Capacity>
class StatsRingBuffer {
    static_assert(Capacity > 0, "StatsRingBuffer capacity must be greater than 0");
    static_assert(std::is_arithmetic<T>::value, "StatsRingBuffer word type must be arithmetic");

public:
    constexpr StatsRingBuffer() noexcept : m_head(0), m_count(0), m_total_pushed(0) {
        for (size_t i = 0; i < Capacity; ++i) {
            m_buffer[i] = T{0};
        }
    }

    // Non-copyable to prevent accidental buffer duplication in memory-constrained MCU
    StatsRingBuffer(const StatsRingBuffer&) = delete;
    StatsRingBuffer& operator=(const StatsRingBuffer&) = delete;

    /**
     * @brief Pushes a new element into the ring buffer.
     * Non-blocking, minimal CPU cycles, ISR-safe.
     */
    inline void push(T value) noexcept {
        size_t head = m_head.load(std::memory_order_relaxed);
        m_buffer[head] = value;
        m_head.store((head + 1) % Capacity, std::memory_order_release);

        size_t count = m_count.load(std::memory_order_relaxed);
        if (count < Capacity) {
            m_count.store(count + 1, std::memory_order_release);
        }
        m_total_pushed.fetch_add(1, std::memory_order_relaxed);
    }

    /**
     * @brief Sets all values in buffer to 'value' and resets count to Capacity.
     */
    void resetTo(T value) noexcept {
        for (size_t i = 0; i < Capacity; ++i) {
            m_buffer[i] = value;
        }
        m_head.store(0, std::memory_order_release);
        m_count.store(Capacity, std::memory_order_release);
    }

    /**
     * @brief Resets all elements in the buffer to 0.
     */
    void clear() noexcept {
        for (size_t i = 0; i < Capacity; ++i) {
            m_buffer[i] = T{0};
        }
        m_head.store(0, std::memory_order_release);
        m_count.store(0, std::memory_order_release);
    }

    inline size_t getCount() const noexcept {
        return m_count.load(std::memory_order_acquire);
    }

    constexpr size_t getCapacity() const noexcept {
        return Capacity;
    }

    inline bool isEmpty() const noexcept {
        return getCount() == 0;
    }

    inline bool isFull() const noexcept {
        return getCount() == Capacity;
    }

    inline uint64_t getTotalPushed() const noexcept {
        return m_total_pushed.load(std::memory_order_relaxed);
    }

    /**
     * @brief Computes average (mean) of the most recent numSamples.
     * @param numSamples Number of recent samples (0 = all available elements).
     */
    float getAvg(size_t numSamples = 0) const noexcept {
        T snapshot[Capacity];
        size_t n = snapshotRecent(snapshot, numSamples);
        if (n == 0) return 0.0f;

        double sum = 0.0;
        for (size_t i = 0; i < n; ++i) {
            sum += static_cast<double>(snapshot[i]);
        }
        return static_cast<float>(sum / static_cast<double>(n));
    }

    /**
     * @brief Computes median of the most recent numSamples.
     * Uses zero-allocation std::nth_element on a local stack snapshot (O(N) time complexity).
     * @param numSamples Number of recent samples (0 = all available elements).
     */
    float getMedian(size_t numSamples = 0) const noexcept {
        T snapshot[Capacity];
        size_t n = snapshotRecent(snapshot, numSamples);
        if (n == 0) return 0.0f;
        if (n == 1) return static_cast<float>(snapshot[0]);

        size_t mid = n / 2;
        std::nth_element(snapshot, snapshot + mid, snapshot + n);
        float median = static_cast<float>(snapshot[mid]);

        if (n % 2 == 0) {
            auto lower = std::max_element(snapshot, snapshot + mid);
            median = (static_cast<float>(*lower) + median) * 0.5f;
        }
        return median;
    }

    /**
     * @brief Computes peak-to-peak range (max - min) of the most recent numSamples.
     */
    float getRange(size_t numSamples = 0) const noexcept {
        T snapshot[Capacity];
        size_t n = snapshotRecent(snapshot, numSamples);
        if (n == 0) return 0.0f;

        auto minmax = std::minmax_element(snapshot, snapshot + n);
        return static_cast<float>(*minmax.second - *minmax.first);
    }

    /**
     * @brief Computes sample standard deviation of the most recent numSamples.
     */
    float getStd(size_t numSamples = 0) const noexcept {
        T snapshot[Capacity];
        size_t n = snapshotRecent(snapshot, numSamples);
        if (n <= 1) return 0.0f;

        double sum = 0.0;
        for (size_t i = 0; i < n; ++i) {
            sum += static_cast<double>(snapshot[i]);
        }
        double mean = sum / static_cast<double>(n);

        double sum_sq_diff = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double diff = static_cast<double>(snapshot[i]) - mean;
            sum_sq_diff += diff * diff;
        }
        return static_cast<float>(std::sqrt(sum_sq_diff / static_cast<double>(n - 1)));
    }

    /**
     * @brief Computes Root-Mean-Square (RMS) of the most recent numSamples.
     */
    float getRMS(size_t numSamples = 0) const noexcept {
        T snapshot[Capacity];
        size_t n = snapshotRecent(snapshot, numSamples);
        if (n == 0) return 0.0f;

        double sum_sq = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double val = static_cast<double>(snapshot[i]);
            sum_sq += val * val;
        }
        return static_cast<float>(std::sqrt(sum_sq / static_cast<double>(n)));
    }

    /**
     * @brief Gets minimum value among the most recent numSamples.
     */
    T getMin(size_t numSamples = 0) const noexcept {
        T snapshot[Capacity];
        size_t n = snapshotRecent(snapshot, numSamples);
        if (n == 0) return T{0};
        return *std::min_element(snapshot, snapshot + n);
    }

    /**
     * @brief Gets maximum value among the most recent numSamples.
     */
    T getMax(size_t numSamples = 0) const noexcept {
        T snapshot[Capacity];
        size_t n = snapshotRecent(snapshot, numSamples);
        if (n == 0) return T{0};
        return *std::max_element(snapshot, snapshot + n);
    }

    /**
     * @brief Copies the most recent samples into an external array in chronological order.
     * @return Number of elements copied.
     */
    size_t copyRecent(T* dest, size_t maxElements) const noexcept {
        if (!dest || maxElements == 0) return 0;
        return snapshotRecent(dest, maxElements);
    }

private:
    /**
     * @brief Atomically snapshots the most recent N elements into dest in chronological order.
     * Zero dynamic memory allocation.
     */
    size_t snapshotRecent(T* dest, size_t requestedSamples) const noexcept {
        size_t head = m_head.load(std::memory_order_acquire);
        size_t count = m_count.load(std::memory_order_acquire);
        if (count == 0) return 0;

        size_t n = (requestedSamples == 0 || requestedSamples > count) ? count : requestedSamples;
        for (size_t i = 0; i < n; ++i) {
            size_t idx = (head + Capacity - n + i) % Capacity;
            dest[i] = m_buffer[idx];
        }
        return n;
    }

    alignas(alignof(T)) T m_buffer[Capacity];
    std::atomic<size_t>   m_head{0};
    std::atomic<size_t>   m_count{0};
    std::atomic<uint64_t> m_total_pushed{0};
};

} // namespace stats

#pragma once

#include <cstdint>
#include <atomic>
#include <type_traits>

namespace stats {

/**
 * @brief Thread-safe counter designed for Single-Producer (SPSC/SPMC) telemetry and health metrics.
 * 
 * Supports configurable word lengths: uint16_t, uint32_t, uint64_t.
 * Increments use relaxed atomic operations for minimal CPU overhead.
 */
template <typename T>
class Counter {
    static_assert(std::is_integral<T>::value && std::is_unsigned<T>::value,
                  "Counter word length must be an unsigned integral type (e.g. uint16_t, uint32_t, uint64_t).");

public:
    constexpr explicit Counter(T initial_val = 0) noexcept : m_value(initial_val) {}

    // Non-copyable to prevent accidental object slicing or race hazards
    Counter(const Counter&) = delete;
    Counter& operator=(const Counter&) = delete;

    /**
     * @brief Increments counter by delta. Non-blocking, ISR-safe.
     */
    inline void increment(T delta = 1) noexcept {
        m_value.fetch_add(delta, std::memory_order_relaxed);
    }

    /**
     * @brief Gets current counter value.
     */
    inline T getValue() const noexcept {
        return m_value.load(std::memory_order_relaxed);
    }

    /**
     * @brief Resets counter to 0.
     */
    inline void reset() noexcept {
        m_value.store(0, std::memory_order_relaxed);
    }

    /**
     * @brief Sets counter to a specific value.
     */
    inline void setValue(T val) noexcept {
        m_value.store(val, std::memory_order_relaxed);
    }

    /**
     * @brief Atomically reads and resets counter to 0.
     * Ideal for 1-second interval periodic delta logging.
     */
    inline T getAndReset() noexcept {
        return m_value.exchange(0, std::memory_order_acq_rel);
    }

    // Operator overloads for ergonomics
    inline Counter& operator++() noexcept {
        increment(1);
        return *this;
    }

    inline Counter& operator+=(T delta) noexcept {
        increment(delta);
        return *this;
    }

    inline explicit operator T() const noexcept {
        return getValue();
    }

private:
    std::atomic<T> m_value{0};
};

// Aliases for common word lengths
using Counter16 = Counter<uint16_t>;
using Counter32 = Counter<uint32_t>;
using Counter64 = Counter<uint64_t>;

} // namespace stats

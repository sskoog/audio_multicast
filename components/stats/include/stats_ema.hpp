#pragma once

#include <atomic>
#include <type_traits>

namespace stats {

/**
 * @brief Exponential Moving Average (EMA) filter.
 * 
 * Formula: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 * Damping factor alpha is set upon creation in range [0.0, 1.0].
 * Automatically initializes internal state to first input sample to eliminate startup settling delay.
 */
template <typename T = float>
class EmaFilter {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value,
                  "EmaFilter value type must be arithmetic (floating-point or integral).");

public:
    explicit EmaFilter(float alpha = 0.05f, T initial_val = 0) noexcept
        : m_alpha(alpha > 1.0f ? 1.0f : (alpha < 0.0f ? 0.0f : alpha)),
          m_value(initial_val),
          m_initialized(false) {}

    EmaFilter(const EmaFilter&) = delete;
    EmaFilter& operator=(const EmaFilter&) = delete;

    /**
     * @brief Updates EMA filter with a new sample.
     */
    void update(T newValue) noexcept {
        float alpha = m_alpha.load(std::memory_order_relaxed);
        float new_val_f = static_cast<float>(newValue);

        if (!m_initialized.load(std::memory_order_relaxed)) {
            m_value.store(new_val_f, std::memory_order_relaxed);
            m_initialized.store(true, std::memory_order_release);
            return;
        }

        float current = m_value.load(std::memory_order_relaxed);
        float updated = (alpha * new_val_f) + ((1.0f - alpha) * current);
        m_value.store(updated, std::memory_order_relaxed);
    }

    /**
     * @brief Gets current filtered value.
     */
    inline T getValue() const noexcept {
        return static_cast<T>(m_value.load(std::memory_order_relaxed));
    }

    /**
     * @brief Resets filter state.
     */
    void reset(T initial_val = 0) noexcept {
        m_value.store(static_cast<float>(initial_val), std::memory_order_relaxed);
        m_initialized.store(false, std::memory_order_release);
    }

    /**
     * @brief Updates damping factor alpha.
     */
    void setAlpha(float alpha) noexcept {
        float clamped = (alpha > 1.0f) ? 1.0f : ((alpha < 0.0f) ? 0.0f : alpha);
        m_alpha.store(clamped, std::memory_order_relaxed);
    }

    inline float getAlpha() const noexcept {
        return m_alpha.load(std::memory_order_relaxed);
    }

    inline bool isInitialized() const noexcept {
        return m_initialized.load(std::memory_order_acquire);
    }

    inline explicit operator T() const noexcept {
        return getValue();
    }

private:
    std::atomic<float> m_alpha{0.05f};
    std::atomic<float> m_value{0.0f};
    std::atomic<bool>  m_initialized{false};
};

using EmaFilterFloat = EmaFilter<float>;

} // namespace stats

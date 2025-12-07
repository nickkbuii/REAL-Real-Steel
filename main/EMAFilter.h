#ifndef EMAFILTER_H
#define EMAFILTER_H

/**
 * @class EMAFilter
 * @brief Generic exponential moving average (EMA) low-pass filter.
 *
 * Implements:
 *      y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *
 * Suitable for acceleration, current, and temperature smoothing.
 *
 * @tparam T Numeric type (float recommended).
 */
template <typename T>
class EMAFilter {
public:
    /**
     * @brief Construct an EMA filter with smoothing factor alpha.
     */
    EMAFilter(T alpha);

    /**
     * @brief Update filter with a new input sample.
     * @return Filtered output.
     */
    T update(T input);

private:
    T alpha;
    T y;
    bool initialized;
};

#endif
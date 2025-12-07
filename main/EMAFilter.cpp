#include "EMAFilter.h"

template <typename T>
EMAFilter<T>::EMAFilter(T alpha)
    : alpha(alpha), y(0), initialized(false)
{}

template <typename T>
T EMAFilter<T>::update(T input) {
    if (!initialized) {
        y = input;
        initialized = true;
    } else {
        y = alpha * input + (static_cast<T>(1) - alpha) * y;
    }
    return y;
}

template class EMAFilter<float>;
template class EMAFilter<double>;
template class EMAFilter<int>;
template class EMAFilter<long>;
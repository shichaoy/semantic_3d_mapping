#ifndef ALGO_UTIL_HPP_ZB5UF8WS
#define ALGO_UTIL_HPP_ZB5UF8WS

#include <cmath>

namespace ca
{

/**
 * Unlike c modulus operator (%), does not return negative numbers.
 * It's more like python modulus.
 * the function mod_wrap(x, y) looks like
 * ^
 * |   /|  /|  /|
 * |  / | / | / |
 * | /  |/  |/  |
 * +---------------> x-axis
 *      -y  0   y
 * note consistency before and after 0.
 *
 */
template<typename T>
T mod_wrap1(T x, T y) {
  return (x - std::floor(static_cast<float>(x)/y)*y);
}

template<typename T>
T mod_wrap2(T x, T y) {
  return ((x % y) + y) % y;
}

template<typename T>
T mod_wrap(T x, T y) {
  return (x < 0) ? (((x % y) + y) % y) : ( x % y );
}

template<typename T>
void inplace_mod_wrap(T& x, T y) {
  x -= (std::floor(static_cast<float>(x)/y)*y);
}

} /* ca */

#endif /* end of include guard: ALGO_UTIL_HPP_ZB5UF8WS */

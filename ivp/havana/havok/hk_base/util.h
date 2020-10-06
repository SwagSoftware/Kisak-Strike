#ifndef HK_BASE_UTIL_H
#define HK_BASE_UTIL_H

template <typename T>
inline void hk_swap( T& a, T& b)
{
    T t(a);
    a = b;
    b = t;
}

#endif /*HK_BASE_UTIL_H*/


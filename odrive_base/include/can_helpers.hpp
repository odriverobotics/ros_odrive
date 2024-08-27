#pragma once

#include <stdint.h>
#include <string.h>

template <typename T>
T can_get_signal_raw(const uint8_t* buf, const size_t startBit, const size_t length, const bool isIntel) {
    constexpr int N = 64;

    union {
        uint64_t tempVal;
        uint8_t tempBuf[N]; // This is used because memcpy into tempVal generates less optimal code
        T retVal;
    };

    const uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;
    const uint8_t shift = isIntel ? startBit : (64 - startBit) - length;

    memcpy(tempBuf, buf, N);
    if (isIntel) {
        tempVal = (tempVal >> shift) & mask;
    } else {
        tempVal = __builtin_bswap64(tempVal);
        tempVal = (tempVal >> shift) & mask;
    }

    return retVal;
}

template <typename T>
void can_set_signal_raw(uint8_t* buf, const T val, const size_t startBit, const size_t length, const bool isIntel) {
    constexpr int N = 8;

    const uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;
    const uint8_t shift = isIntel ? startBit : (64 - startBit) - length;

    union {
        uint64_t valAsBits;
        T tempVal;
    };

    tempVal = val;

    union {
        uint64_t data;
        uint8_t tempBuf[N];
    };

    memcpy(tempBuf, buf, N);
    if (isIntel) {
        data &= ~(mask << shift);
        data |= valAsBits << shift;
    } else {
        data = __builtin_bswap64(data);
        data &= ~(mask << shift);
        data |= valAsBits << shift;
        data = __builtin_bswap64(data);
    }

    memcpy(buf, tempBuf, N);
}

template <typename T>
float can_get_signal_raw(
    const uint8_t* buf,
    const size_t startBit,
    const size_t length,
    const bool isIntel,
    const float factor,
    const float offset
) {
    T retVal = can_get_signal_raw<T>(buf, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

template <typename T>
constexpr void can_set_signal_raw(
    uint8_t* buf,
    const float val,
    const size_t startBit,
    const size_t length,
    const bool isIntel,
    const float factor,
    const float offset
) {
    T scaledVal = static_cast<T>((val - offset) / factor);
    can_set_signal_raw<T>(buf, scaledVal, startBit, length, isIntel);
}

template <typename T, typename TMsg>
constexpr T can_get_signal(const TMsg& msg, const size_t startBit, const size_t length, const bool isIntel) {
    return can_get_signal_raw<T>(can_msg_get_payload(msg).data(), startBit, length, isIntel);
}

template <typename T, typename TMsg>
constexpr void can_set_signal(TMsg& msg, const T val, const size_t startBit, const size_t length, const bool isIntel) {
    can_set_signal_raw<T>(can_msg_get_payload(msg).data(), val, startBit, length, isIntel);
}
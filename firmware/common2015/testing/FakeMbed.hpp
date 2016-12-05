#pragma once

#include "mbed.h"

/**
 * This file provides some helper classes for emulating an mbed's hardware.  It
 * is useful for unit-testing code on a computer when an mbed is not available
 * or convenient.
 */
namespace fake_mbed {

class DigitalIn {
public:
    DigitalIn(int value) : m_value(value) {}

    int read() const { return m_value; }
    operator int() { return read(); }

private:
    int m_value;
};


class SPI {
public:
    PinName m_mosi = NC;
    PinName m_miso = NC;
    PinName m_sclk = NC
    PinName m_ssel = NC;

    SPI(PinName mosi, PinName miso, PinName sclk, PinName ssel = NC)
    : m_mosi(mosi), m_miso(miso), m_sclk(sclk), m_ssel(ssel) {}

    void format(int bits, int mode = 0) { m_bits = bits; m_mode = mode; }

    void frequency(int hz = 1000000) { m_hz = hz }

    int write(int value)
    {

    }

private:
    int m_bits;
    int m_mode;
    int m_hz;
};

}   // namespace fake_mbed

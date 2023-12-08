# Adam's USB Implementation

This is a USB stack for embedded devices.

I wrote it in part due to some shortcomings of tinyusb, and as a hobby project.
This only supports a few device classes and hardware platforms.  If you are
looking for a more full-featured, robust USB stack you probably should use
[tinyusb](https://github.com/hathach/tinyusb) instead.  tinyusb is much more
mature, and is officially supported by several vendors.


## Motivation

The main motivation for this library was to have a sane
synchronization/concurrency story on ESP32 platforms.  I don't trust the
concurrency model that tinyusb uses on FreeRTOS.  To be fair, it appears like
tinyusb was designed for simple single-core MCUs, and FreeRTOS support was
added later.  However, the model they use in their example code, with USB state
be manipulated from many tasks possibly simultaneously, doesn't seem safe,
particularly on multi-core MCUs.  When I've submitted some bug reports
Espressif has indicated that they are working on a re-write of the tinyusb DCD
implementation for ESP32.  In the meantime, I figured I might as well tackle
just building my own stack.

## C++

AUSB is implemented in C++.  It doesn't use RTII or exceptions, since it is
intended to also play well when used with pure-C code.

Compared to pure C, C++ provides a number of additional convenience
functionality.  C++ also allows writing `constexpr` functions that can perform
compile-time construction of USB descriptors, which can then be embedded in the
firmware's read-only data section.  This seems nicer than using C macros or a
completely separate build-time tool for generating descriptors.

I did consider using Rust, since Rust has growing support for embedded devices.
However, one factor that caused me to not use Rust is that I do have some older
projects using 8-bit AVR MCUs (atmega32U4), and from a cursory investigation it
seems unlikely that Rust will be usable on these 8-bit microcontrollers.

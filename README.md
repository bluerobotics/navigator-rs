# Navigator-rs

This crate serves as the entry point for embedding applications using Rust on Blue Robotics's [Navigator](https://bluerobotics.com/store/comm-control-power/control/navigator/).

The Navigator board has the Raspberry Pi HAT form factor, which allows you to easily attach it to a Raspberry Pi 4 board. Then you can unleash the power of Navigator to develop new solutions or interact with your ROV hardware.

The board offers the following capabilities:

Control:
- LEDs
- PWM (Pulse Width Modulation) with 16 channels

Measurements:
- ADC (Analog Digital Converter) entries
- Magnetic field
- Acceleration
- Angular velocity
- Temperature
- Pressure

Currently, it supports **armv7** and **aarch64** architectures, which are the official defaults for [BlueOS](https://docs.bluerobotics.com/ardusub-zola/software/onboard/BlueOS-1.1/).
However, despite using the embedded-hal concept, new ports can be added as long as the platform matches the hardware design and specifications.

For more information, including installation instructions, schematics, and hardware specifications, please check the [navigator hardware setup guide](https://bluerobotics.com/learn/navigator-hardware-setup/#introduction).

## How to use this crate:
The capabilities of the Navigator can be accessed through the Navigator object. Please check the examples in the **[`Implementations`](https://docs.bluerobotics.com/navigator-rs/navigator_rs/struct.Navigator.html#implementations)** section.
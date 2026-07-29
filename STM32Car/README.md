# STM32Car Firmware

STM32H745ZI CM7 firmware for the smart car chassis, built with IAR EWARM and STM32H7 HAL.

## Current Status

- TJC touch panel is connected to USART2: TX PD5, RX PD6, 115200 8N1.
- TJC stop key sends `5A A5 00 FF` and enters stop mode.
- TJC line key sends `5A A5 0D F2` and enters grayscale line mode.
- TJC query key/value `5A A5 0E F1` returns the current mode.
- The old camera UART, camera line protocol, and camera line-follow control have been removed from the application project.
- Four-channel grayscale line following is now the intended line sensor path, but its pins and polarity are not wired in this firmware yet. Until that hardware mapping is added, grayscale line mode keeps the motors stopped.
- Motor, encoder, IMU heading, wheel-speed PID, and closed-loop stop handling remain in place.

## Hardware Map

| Function | Pins |
| --- | --- |
| TJC UART | USART2 TX PD5, RX PD6 |
| Left motor PWM/DIR | TIM2 CH3 PA2, DIR PA3 |
| Right motor PWM/DIR | TIM2 CH1 PA0, DIR PA1 |
| Left encoder | TIM8 CH1 PC6, TIM8 CH2 PC7 |
| Right encoder | TIM1 CH1 PA8, TIM1 CH2 PE11 |

## TJC Protocol

The touch panel sends a 4-byte frame:

```text
5A A5 <mode> <check>
```

The checksum byte is the bitwise inverse of the mode byte:

```text
check = mode ^ 0xFF
```

Supported mode values:

| Mode | Frame | Meaning |
| --- | --- | --- |
| `0x00` | `5A A5 00 FF` | Stop |
| `0x0D` | `5A A5 0D F2` | Four-channel grayscale line mode |
| `0x0E` | `5A A5 0E F1` | Query current mode |

The firmware replies through USART2 with a 5-byte result frame: `A5 <result> <current_mode> <request> <xor>`.

## Next Hardware Step

Add the four grayscale sensor interface once the wiring is fixed:

- Sensor type: digital DO or analog AO.
- Four channel pins, from left to right.
- Active polarity for black line detection.
- Desired steering behavior when all channels lose the line.

After that, `APP_MODE_GRAY_LINE` can drive the motor speed and steering from the grayscale sensor state instead of holding the car stopped.



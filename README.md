This is the dummy MC test code for the control board (2026-2027).

- CAN Watchdog is disabled for this version to simplify testing logic
- Build for either STM using `#define CAN_BENCH_ROLE CAN_BENCH_ROLE_DUMMY` or `#define CAN_BENCH_ROLE CAN_BENCH_ROLE_CONTROLLER` for dummy MC STM or Control Board STM

To test:

- Connect two STM32 NUCLEO H723ZG boards via CAN tranceivers using their `PD0` and `PD1` pins
- Connect potentiometer via `3.3V` and `GND`, with output going to `CONTROLLER`'s PA3 pin
- Can monitor serial output of either or both boards, for debugging purposes
- Since the `DUMMY` STM acts as the motor controller, when its red LED flashes it means the "MC" is in regen mode, and when its green LED flashes it means the "MC" is in throttle. This will eventually be translated to our actual MC. 

To do next:

- Add pedal (potentiometer) input calibration, increase deadzone for neutral mode 


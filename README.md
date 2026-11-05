# rtnav

Bare-metal C firmware for an ATmega328P (Arduino Uno) line-following and
obstacle-avoiding robot: a 100 Hz timer-interrupt control loop, interrupt-driven ADC
sensing of IR sensors, PWM motor drive and a navigation state machine, written against
the registers with no Arduino library.

This is a re-implementation of a robot I built earlier on Arduino hardware. This
version is verified in simulation, including running the compiled firmware in a
cycle-accurate AVR simulator; it is not measured on physical hardware.

> Work in progress. Design, results and usage will be documented here as the
> milestones land.

## Build the firmware

Needs `avr-gcc` and `avr-libc`, or Docker:

```sh
docker build -t rtnav-dev docker
docker run --rm -v "$PWD":/src -w /src rtnav-dev make -C firmware
```

## License

MIT, see [LICENSE](LICENSE).

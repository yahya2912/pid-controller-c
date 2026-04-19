# PID Controller in C

A lightweight PID controller implementation in C, simulating a temperature
control system. Built as a learning project to understand closed-loop control
systems from theory to code.

## Theory

A PID controller corrects the deviation between a desired value (setpoint)
and a measured value (measurement) using three terms:

- **P (Proportional):** Reacts to the current error. Larger error = larger correction.
- **I (Integral):** Accumulates past errors to eliminate steady-state offset.
- **D (Derivative):** Reacts to the rate of change of the error to reduce overshoot.

Output: `y = Kp*e + Ki*∑e*dt + Kd*(de/dt)`

## Project Structure
pid-controller-c/
├── include/pid.h       # PID struct and function declarations
├── src/pid.c           # PID implementation (init, update, anti-windup)
├── src/main.c          # Temperature simulation loop
├── test/test_pid.c     # Unit tests (coming soon)
└── CMakeLists.txt      # Build configuration

## Build & Run

```bash
mkdir build && cd build
cmake ..
make
./pid_sim
```

## Tuning

| Parameter | Effect |
|-----------|--------|
| Kp | Speed of response. Too high = oscillation. |
| Ki | Eliminates steady-state error. Too high = overshoot. |
| Kd | Reduces overshoot. Too high = instability. |

## Next Steps

- Unit tests in test/test_pid.c
- Port to STM32 / Arduino
- Second-order plant model

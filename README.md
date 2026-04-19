# PID Controller in C

A lightweight PID controller implementation in C, simulating a temperature
control system.

## Theory

A PID controller corrects the deviation between a desired value (setpoint)
and a measured value (measurement) using three terms:

- **P (Proportional):** Reacts to the current error. Larger error = larger correction.
- **I (Integral):** Accumulates past errors to eliminate steady-state offset.
- **D (Derivative):** Reacts to the rate of change of the error to reduce overshoot.

Output: `y = Kp*e + Ki*∑e*dt + Kd*(de/dt)`

## Project Structure

```
pid-controller-c/
├── include/pid.h       # PID struct and function declarations
├── src/pid.c           # PID implementation (init, update, anti-windup)
├── src/main.c          # Temperature simulation loop
├── src/gui.c           # GTK3 desktop UI with live response curve
├── test/test_pid.c     # Unit tests (coming soon)
└── CMakeLists.txt      # Build configuration
```

## Dependencies
- GCC
- CMake 3.16+
- GTK3 (for the GUI)

```bash
sudo apt install cmake gcc libgtk-3-dev
```

## Build & Run

**CLI simulation:**
```bash
mkdir build && cd build
cmake ..
make
./pid_sim
```

**GTK GUI:**
```bash
cd build
make pid_gui
./pid_gui
```

## GUI Usage
1. Set **Kp**, **Ki**, **Kd** using the input fields
2. Set a **Setpoint** (target value)
3. Click **Run Simulation**
4. The response curve shows:
   - 🟢 Green line — Setpoint (target)
   - 🔵 Blue curve — Process Variable (system response)

## Tuning
| Parameter | Effect |
|-----------|--------|
| Kp | Speed of response. Too high = oscillation. |
| Ki | Eliminates steady-state error. Too high = overshoot. |
| Kd | Reduces overshoot. Too high = instability. |
EOF

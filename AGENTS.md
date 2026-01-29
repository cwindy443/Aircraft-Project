# AGENTS.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

This is an aircraft flight control system project, developing control algorithms for an aircraft model. The codebase includes:

- **Flight control algorithms**: Implementation of pitch, roll, and yaw control for a canard-configured aircraft (J-10C inspired)
- **Mixed language codebase**: Primarily C/C++ for flight control, with exploratory Ada code
- **Control surface mixing**: Implements differential control for canards, elevons (combined elevator/aileron), and rudder

## Key Architecture Concepts

### Flight Control System (Flying-Control/10c.c)

The main flight control implementation uses a feedback control system with:

1. **Statically unstable aircraft compensation**: The aircraft is inherently unstable and requires continuous feedback control to maintain stable flight
2. **Control mixing**: Elevons serve dual purpose for both pitch and roll control through differential mixing
3. **Rate damping**: Uses pitch_rate, roll_rate, and yaw_rate feedback to stabilize the aircraft
4. **Safety limiting**: Hard limits on control surface deflections to prevent structural damage

Core data flow:
```
PilotInput → flight_control_law() → ActuatorOutput
             ↑
        AircraftState (sensor feedback)
```

The `flight_control_law()` function runs in a high-frequency loop (100-400 Hz in real systems) to continuously adjust control surfaces based on pilot input and aircraft state.

## Development Commands

### Building C/C++ Code

Compile the main flight control system:
```bash
gcc -o Flying-Control/plane Flying-Control/10c.c -lm
```

Compile C++ code:
```bash
g++ -o Flying-Control/plane Flying-Control/plane.cpp
```

### Running

Execute the flight control simulation:
```bash
./Flying-Control/plane
```

### Ada Development

Ada toolchain (gnatmake/gprbuild) is not currently installed on this system. To work with Ada files like `test.ada`:
- Install GNAT toolchain first
- Compile: `gnatmake test.ada`
- Run: `./hello`

## Important Notes

- The C code includes Chinese comments explaining the flight control algorithms
- Control gains (e.g., 25.0, 12.5, 20.0) are tuned for specific aircraft dynamics and should be modified carefully
- The plane.cpp file contains a recursive initialization bug (line 5: `init()` calls itself)

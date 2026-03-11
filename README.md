# Automatic Farm Robot (Elma)

This project implements an **autonomous farm robot** control system using the [Elma](https://github.com/klavinslab/elma) event loop and process manager. The robot navigates aisles, visits checkpoints, detects AprilTags and boxes, coordinates with a robot arm and vision (planting/harvest) modules, and supports charging-station logic—all driven by a finite state machine (FSM) running in Elma at 1 ms period for 30 seconds.

## Overview

**Goal:** Build a C++ application on top of Elma that demonstrates substantial reactive behavior: a farm robot that cruises, reacts to checkpoints and AprilTag detections, performs box and task states, and can drive to a charging station. The design supports both **mock** modules (for testing and grading without hardware) and **real** modules (e.g. ElintTech vehicle and checkpoint sensor over serial).

Key components:

- **Elma** library: event loop, processes, state machines, channels (see [Elma on Github](https://github.com/klavinslab/elma)).
- **Farm robot application** (`farm_robot/`): main entry point, shared state, FSM (cruising, reach checkpoint, box, charging, etc.), and controllers for vehicle, checkpoint sensor, robot arm, and communicator.
- **Modules** (pluggable): vehicle, checkpoint sensor, AprilTag sensor, robot arm, planting detector, harvest detector—each with mock and (where applicable) ElintTech implementations.

## Key Challenges and How They Were Addressed

1. **Coordinating FSM with shared state and multiple controllers**  
   The FSM needs to read/write shared state and trigger controller actions (e.g. “go to next checkpoint”). This was addressed by introducing a single `SharedState` object (thread-safe where needed) and passing pointers to it and to the relevant controllers into the FSM. State classes use `FsmContext` to access shared state and controllers and emit events to drive transitions.

2. **Abstracting mock vs real hardware**  
   The same FSM and controllers must run with mock modules (for grading and tests) or real hardware (ElintTech over serial). This was addressed by defining interface classes (e.g. `IVehicleModule`, `ICheckpointSensorModule`) and factory functions in `main.cc` that choose mock or ElintTech implementation based on config (`Config::VEHICLE_MODULE_TYPE`, etc.), so the rest of the code depends only on the interfaces.

3. **Serial I/O and threading**  
   Real vehicle and checkpoint sensor communicate over serial ports. Their implementations handle serial open/read/write and any threading internally so the Elma process callbacks stay non-blocking and the event loop continues to run at the scheduled period.

4. **State machine structure and transitions**  
   Keeping the FSM readable and maintainable with many states and events was addressed by grouping state classes in one file (`robot_state_machine.cc`), using a single `FsmContext` struct for dependencies, and naming events and states after the rubric behavior (cruising, reach_checking_point, box_on_position, charging, etc.).

## Installation and Running (Docker)

Assume Docker is installed. Use the **exact** command below to run the grading environment (mounts the repo at `/source`, no hardware device):

```bash
docker run --rm -v $PWD:/source -it klavins/elma:latest bash
```

Inside the container, from the repo root (e.g. `/source` if you mounted the project there):

```bash
make
```

This builds the Elma library (`lib/libelma.a`), tests, examples, and the farm robot binary. To confirm a clean build:

```bash
make clean
make
```

**Note:** Building requires the dependencies provided in the Docker image (e.g. `json`, OpenSSL). If `make` fails on your host (e.g. missing `json/json.h`), run the same commands inside the container above.

**Hardware users:** To use the real vehicle/serial device, run with device passthrough and optional container name, for example:

```bash
docker run --rm -v $PWD:/source --device /dev/ttyUSB0:/dev/ttyUSB0 -it klavins/elma:latest bash
```

Then set config to use `elintech` modules (e.g. in `include/farm_robot/cores/config.h` or runtime config if added) so the factories in `main.cc` create ElintTech implementations.

## How to Run and Use the Project

After building with `make`:

- From the **farm_robot** directory:
  ```bash
  ./bin/main
  ```
- Or from the **repo root**:
  ```bash
  ./farm_robot/bin/main
  ```

**Expected behavior:**

- The process runs for **30 seconds** (or until you interrupt it).
- By default, all modules are **mock** (no real hardware required). You should see console output for:
  - Initialization (“Getting all the modules from container…”, “Initializing controller…”).
  - State transitions and FSM behavior (cruising, checkpoint reached, box detection, charging, etc., depending on mock responses).
- The main executable is `farm_robot/bin/main`; it links against the Elma library and all farm_robot application and module code.

To run only the farm robot build (skip tests and examples):

```bash
make farm_robot
./farm_robot/bin/main
```

## Acknowledgments

- **Elma** – Event loop and process manager: [klavinslab/elma](https://github.com/klavinslab/elma).
- **ECEP520** – University of Washington course and materials: [ECEP520](https://github.com/klavins/ECEP520) (event loop management, events and finite state machines, HTTP in event loops).
- **Google Test** – [googletest](https://github.com/google/googletest).
- **nlohmann/json** – [nlohmann/json](https://github.com/nlohmann/json).
- **cpp-httplib** – [yhirose/cpp-httplib](https://github.com/yhirose/cpp-httplib) and [klavins/cpp-httplib](https://github.com/klavins/cpp-httplib) fork.

---

## Elma (Library)

Elma is an event loop and process manager for embedded and reactive systems. It is structured as a C++ library on top of which this project is built. It manages processes, finite state machines, events, and communication channels at specified frequencies. More information:

- [Event loop management](https://github.com/klavins/ECEP520/tree/master/week_6)
- [Events and finite state machines](https://github.com/klavins/ECEP520/tree/master/week_7)
- [HTTP in event loops](https://github.com/klavins/ECEP520/blob/master/week_8)

See the `examples/` directory for small Elma examples (e.g. `examples/bin/basic` after `make`).

## License

This project uses the MIT license. See [LICENSE](LICENSE) or [LICENSE.md](LICENSE.md) for the full text.

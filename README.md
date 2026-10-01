# V2X Multi-Criteria Emergency Dispatch (C++17, console edition)

A deterministic discrete-event simulator that dispatches ambulances and fire
engines on a road network while V2X hazards, incident severity and fleet
availability change. Pure C++17 - no external libraries, no web UI.

## Build and run (Windows, Visual Studio / MSVC)

Open a *Developer Command Prompt for VS* in this folder:

    build.bat                                  :: configure, compile, run tests
    build\Release\emergency_dispatch.exe       :: interactive menu
    run_demo.bat                               :: all 5 scripted scenarios

Manual equivalent: `cmake -S . -B build`, `cmake --build build --config Release`,
`cd build && ctest -C Release --output-on-failure`.
Any C++17 compiler works too (`g++ -std=c++17 -Iinclude src/*.cpp -o dispatch`).

Command line: `emergency_dispatch.exe --demo | --scenario N | --help`.
`docs/sample_demo_output.txt` holds a full transcript of `--demo`.

## Layout

| File | Responsibility |
|---|---|
| `include/types.hpp` | ids, enums (vehicle lifecycle states, incident types ...) |
| `road_network` | nodes, edges, V2X hazards, travel-time formula |
| `router` | Dijkstra on dynamic costs, perimeter staging node search |
| `entities.hpp` | Vehicle, Incident, Analytics, `DispatchPolicy` (all weights) |
| `world` / `motion` | shared state + low-level movement helpers |
| `dispatcher` | suitability scoring, co-dispatch, preemption, queue aging |
| `simulation` | discrete-event engine and vehicle state machine |
| `telemetry` | console dashboard + JSON snapshot (`toJson`) |
| `scenario` | demo city, fleet and the five scripted scenarios |
| `tests/test_main.cpp` | 21 tests / 135 checks, registered with CTest |

## Model summary

* **Edge time** = `distance_km / speed_kmh * congestion * 60` minutes; closed edges are unreachable.
* **Suitability** (higher is better) = `-wTime*ETA` + severity-scaled triage / paramedic (ambulances) or water / ladder (engines) - small penalty for units still returning to base.
* **Co-dispatch**: Fire/Rescue severity >= 4 needs 1 engine + 1 ambulance, committed all-or-nothing.
* **Preemption**: a severity-5 call may reclaim an en-route/staged unit from a severity <= 2 call; the displaced call returns to the queue.
* **Queue aging**: effective priority = severity + 0.1 per waiting minute; a starvation guard stops lower-priority calls from taking roles a higher-priority call is still waiting for.
* **Perimeter staging**: if the incident is cut off, the unit drives to the reachable node closest to it, waits, and resumes automatically when the road reopens.
* **Rerouting**: units re-plan at every junction, so closures/congestion injected mid-trip change their path.
* **Signal preemption**: severity >= 4 responses run at 0.85x hop time.

All coefficients are simulation heuristics (see `DispatchPolicy`), not
empirically validated emergency-service values.

## Lifecycle

`IDLE_STATION -> EN_ROUTE_INCIDENT -> ON_SCENE -> TRANSPORTING_HOSPITAL -> AT_HOSPITAL_TURNOVER -> RETURNING_TO_BASE -> IDLE_STATION`
(engines skip the hospital legs; `STAGED_AT_PERIMETER` is added for cut-off incidents).

#pragma once
// routevis.hpp  –  Interactive step-by-step route visualiser
//
// Given a source and destination node in the demo city, computes the
// shortest (or A*) path and lets the user "drive" the emergency vehicle
// hop-by-hop while watching an ASCII map update in real time.
//
// Accessible three ways:
//   1. Menu option 11 inside the interactive console.
//   2. Command-line flag:  emergency_dispatch --route
//   3. Directly from code: routeVisualize(sim, from, to, out, pause).

#include <ostream>
#include "simulation.hpp"

namespace v2x {

// Full interactive session: asks the user to pick source and destination,
// then animates the route step-by-step.  Returns false on end-of-input.
bool runRouteVisualizer(Simulation& sim, std::ostream& out);

// Non-interactive: print the route and the animated walk to `out`.
// `pauseMs` – milliseconds to wait between hops (0 = no pause / batch mode).
void routeVisualize(const Simulation& sim, NodeId from, NodeId to,
                    std::ostream& out, int pauseMs = 800);

}  // namespace v2x

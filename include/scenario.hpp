#pragma once
#include <ostream>

#include "simulation.hpp"

namespace v2x {

// Node ids of the demo city (same order as buildDemoCity()).
namespace city {
enum : NodeId {
    Central = 0, NorthHub, EastHub, SouthHub, WestHub, Riverside, OldTown, Harbor, Industrial,
    StationAlpha, StationBravo, StationCharlie, CityGeneral, EastsideMedical, Hillcrest
};
}

// 15-node demo city, 4 ambulances (mixed capability) and 3 fire engines.
CityData buildDemoCity();

constexpr int kScenarioCount = 5;
const char* scenarioTitle(int number);  // 1-based
// Runs a scripted scenario on a fresh simulation and prints the result.
void runScenario(int number, std::ostream& out);

}  // namespace v2x

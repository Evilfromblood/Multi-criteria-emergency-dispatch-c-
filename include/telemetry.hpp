#pragma once
#include <ostream>
#include <string>

#include "world.hpp"

namespace v2x {

// Full topology / fleet / incident / hazard / KPI snapshot as JSON text.
std::string toJson(const World& world);

void printFleet(std::ostream& out, const World& world);
void printIncidents(std::ostream& out, const World& world);
void printHazards(std::ostream& out, const World& world);
void printKpis(std::ostream& out, const World& world);
void printDashboard(std::ostream& out, const World& world);
void printNetwork(std::ostream& out, const World& world);

}  // namespace v2x

#pragma once
#include <deque>
#include <ostream>
#include <string>
#include <vector>

#include "entities.hpp"
#include "road_network.hpp"

namespace v2x {

// Static description of a scenario: the map plus the fleet.
struct CityData {
    RoadNetwork net;
    std::vector<Vehicle> fleet;
};

// All mutable simulation state. Shared by the dispatcher and the simulation.
struct World {
    RoadNetwork net;
    std::vector<Vehicle> fleet;       // fleet[i].id == i
    std::deque<Incident> incidents;   // incidents[i].id == i (deque: stable references)
    std::vector<IncidentId> queue;    // incidents waiting for units
    DispatchPolicy policy;
    Analytics stats;
    double now = 0.0;                 // simulated minutes

    std::vector<std::string> eventLog;
    std::ostream* echo = nullptr;     // when set, events are printed live

    Vehicle& vehicle(VehicleId id) { return fleet.at(static_cast<std::size_t>(id)); }
    const Vehicle& vehicle(VehicleId id) const { return fleet.at(static_cast<std::size_t>(id)); }
    Incident& incident(IncidentId id) { return incidents.at(static_cast<std::size_t>(id)); }
    const Incident& incident(IncidentId id) const {
        return incidents.at(static_cast<std::size_t>(id));
    }

    std::string nodeName(NodeId id) const;
    void log(const std::string& message);
};

}  // namespace v2x

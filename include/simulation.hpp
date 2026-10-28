#pragma once
#include <functional>
#include <map>
#include <ostream>
#include <string>

#include "dispatcher.hpp"

namespace v2x {

// Discrete-event engine: simulated time jumps from event to event
// (hop completion, on-scene timer, scheduled action) - never by wall-clock.
class Simulation {
public:
    explicit Simulation(CityData city, DispatchPolicy policy = DispatchPolicy());
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    World& world() { return world_; }
    const World& world() const { return world_; }
    Dispatcher& dispatcher() { return dispatcher_; }
    const Dispatcher& dispatcher() const { return dispatcher_; }
    double now() const { return world_.now; }
    void setEcho(std::ostream* out) { world_.echo = out; }

    IncidentId createIncident(IncidentType type, int severity, NodeId location,
                              const std::string& description = "");
    HazardId injectHazard(EdgeId edge, HazardKind kind, double multiplier,
                          const std::string& description = "");
    bool resolveHazard(HazardId id);

    // Run `action` when simulated time reaches `atTime` (absolute minutes).
    void schedule(double atTime, const std::string& label,
                  std::function<void(Simulation&)> action);

    void advance(double minutes);
    bool allQuiet() const;  // every incident resolved and every unit idle at base
    // Advance until quiet or until `maxMinutes` have elapsed; returns true if quiet.
    bool runUntilQuiet(double maxMinutes);
    void reset();

    NodeId chooseHospital(NodeId from) const;

private:
    struct Scheduled {
        std::string label;
        std::function<void(Simulation&)> action;
    };

    void load(const CityData& city);
    void afterEvents();       // settle, retry blocked units, process the queue
    void settle();            // zero-time transitions
    void retryBlocked();      // held units retry, staged units resume
    void onNodeReached(Vehicle& v);
    bool replan(Vehicle& v);
    void completeHop(Vehicle& v);
    void completeTimer(Vehicle& v);
    void arriveOnScene(Vehicle& v);
    void completeScene(Vehicle& v);
    void resolveIncident(Incident& inc);
    void rescaleForEdge(EdgeId edge);
    double sceneMinutes(const Vehicle& v, const Incident& inc) const;
    double nextEventDelay() const;

    CityData initial_;
    World world_;
    Dispatcher dispatcher_;
    std::multimap<double, Scheduled> schedule_;
};

}  // namespace v2x

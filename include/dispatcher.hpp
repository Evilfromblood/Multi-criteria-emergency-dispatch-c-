#pragma once
#include <set>
#include <string>
#include <vector>

#include "motion.hpp"

namespace v2x {

// One way of serving an incident role with one unit.
struct Candidate {
    VehicleId vehicle = -1;
    Route route;               // from the unit's anchor node
    bool viaStaging = false;   // incident unreachable: route ends at a staging node
    NodeId stagingNode = -1;
    bool preempt = false;      // unit would be reclaimed from a low-severity call
    double eta = kInf;         // minutes (including staging penalty if any)
    double score = -kInf;      // suitability, higher is better
};

class Dispatcher {
public:
    explicit Dispatcher(World& world) : w_(world) {}

    // Units needed: Fire/Rescue = 1 engine, plus 1 ambulance when severity >= 4.
    static std::vector<VehicleType> requiredRoles(IncidentType type, int severity);

    // Multi-criteria suitability heuristic (see DispatchPolicy for the weights).
    double suitability(const Vehicle& v, const Incident& inc, double etaMinutes) const;

    // severity + aging; used to order the waiting queue.
    double effectivePriority(const Incident& inc) const;

    // Ranked candidates (best first) for a single role.
    std::vector<Candidate> rank(const Incident& inc, VehicleType role,
                                const std::set<VehicleId>& exclude,
                                bool includePreemptable) const;

    // "What would the dispatcher pick?" without creating an incident.
    std::vector<Candidate> previewRanking(IncidentType type, int severity, NodeId location,
                                          VehicleType role) const;

    // Register a new incident and try to dispatch it immediately.
    IncidentId submit(IncidentType type, int severity, NodeId location,
                      const std::string& description);

    // Re-attempt queued incidents (priority order, starvation-safe).
    void processQueue();

private:
    bool tryDispatch(Incident& inc, bool allowPreempt);
    void assign(Incident& inc, const Candidate& c);
    void releaseForPreemption(Vehicle& v, const Incident& by);
    void enqueue(Incident& inc);

    World& w_;
};

}  // namespace v2x

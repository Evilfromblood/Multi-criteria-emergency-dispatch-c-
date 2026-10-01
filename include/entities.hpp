#pragma once
#include <deque>
#include <string>
#include <vector>

#include "types.hpp"

namespace v2x {

// Every coefficient here is a *simulation heuristic*, exposed as policy.
struct DispatchPolicy {
    // --- suitability weights ---
    double wTime = 1.0;                  // penalty per minute of ETA
    double wTriage = 2.0;                // ambulance triage level (1..5), scaled by severity
    double wParamedic = 4.0;             // paramedic on board, scaled by severity
    double wWater = 6.0;                 // fire-engine water capacity, scaled by severity (fires)
    double wLadder = 5.0;                // ladder bonus for rescue / severe incidents
    double referenceWaterLiters = 4000.0;
    double returningPenalty = 3.0;       // unit still returning to base is slightly less attractive
    double unreachablePenaltyMinutes = 15.0;  // extra ETA charged when only staging is possible
    // --- policy switches ---
    bool interceptReturning = true;      // returning units may be intercepted
    int preemptMinSeverity = 5;          // incidents at/above this level may preempt
    int preemptMaxSeverity = 2;          // only calls at/below this level can be displaced
    double agingPerMinute = 0.1;         // queue aging: priority gained per waiting minute
    int signalPreemptionMinSeverity = 4; // traffic-signal preemption for severe calls
    double signalPreemptionFactor = 0.85;  // hop-time multiplier when signals are preempted
    double hospitalTurnoverMinutes = 10.0;
};

struct Vehicle {
    VehicleId id = -1;
    std::string callsign;
    VehicleType type = VehicleType::Ambulance;
    NodeId base = 0;

    // capability profile
    int triageLevel = 0;       // ambulances: 1 (BLS) .. 5 (advanced trauma)
    bool paramedic = false;    // ambulances
    double waterLiters = 0.0;  // fire engines
    bool ladder = false;       // fire engines

    // lifecycle state
    VehicleState state = VehicleState::IdleStation;
    IncidentId incident = -1;
    NodeId destination = -1;
    bool headingToStaging = false;
    bool holding = false;      // blocked with no route; retried on every event
    double timer = 0.0;        // remaining minutes for ON_SCENE / AT_HOSPITAL
    double speedFactor = 1.0;  // < 1 when traffic signals are preempted

    // position: at `node`, or travelling node -> nextNode along hopEdge
    NodeId node = 0;
    bool midHop = false;
    NodeId nextNode = -1;
    EdgeId hopEdge = -1;
    double hopRemaining = 0.0;  // minutes left on current hop
    double hopRate = 0.0;       // full-hop minutes at the current conditions
    std::deque<NodeId> plan;    // nodes still to visit after node / nextNode

    // statistics
    double distanceKm = 0.0;
    double busyMinutes = 0.0;
    int dispatches = 0;
};

struct Incident {
    IncidentId id = -1;
    IncidentType type = IncidentType::Medical;
    int severity = 1;  // 1..5
    NodeId location = 0;
    std::string description;
    IncidentStatus status = IncidentStatus::Queued;

    double createdAt = 0.0;
    double queuedSince = -1.0;  // -1 when not queued
    double firstDispatchAt = -1.0;
    double firstArrivalAt = -1.0;
    double resolvedAt = -1.0;
    int preemptedCount = 0;

    std::vector<VehicleId> assigned;   // units currently committed
    std::vector<VehicleId> completed;  // units that finished on scene
};

struct Analytics {
    int incidentsCreated = 0;
    int incidentsResolved = 0;
    int dispatches = 0;      // unit assignments
    int coDispatches = 0;    // heterogeneous packages
    int preemptions = 0;
    int stagings = 0;
    int resumes = 0;
    int reroutes = 0;
    int maxQueueDepth = 0;
    double totalResponseMinutes = 0.0;
    int responseSamples = 0;
    double totalQueueWaitMinutes = 0.0;
    int queueWaitSamples = 0;

    double meanResponse() const {
        return responseSamples ? totalResponseMinutes / responseSamples : 0.0;
    }
    double meanQueueWait() const {
        return queueWaitSamples ? totalQueueWaitMinutes / queueWaitSamples : 0.0;
    }
};

}  // namespace v2x

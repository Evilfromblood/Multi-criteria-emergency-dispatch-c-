#include "dispatcher.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>

namespace v2x {

std::vector<VehicleType> Dispatcher::requiredRoles(IncidentType type, int severity) {
    switch (type) {
        case IncidentType::Medical:
            return {VehicleType::Ambulance};
        case IncidentType::Fire:
        case IncidentType::Rescue:
            if (severity >= 4) return {VehicleType::FireEngine, VehicleType::Ambulance};
            return {VehicleType::FireEngine};
    }
    return {};
}

double Dispatcher::suitability(const Vehicle& v, const Incident& inc, double eta) const {
    const DispatchPolicy& p = w_.policy;
    const double sev = inc.severity / 5.0;
    double s = -p.wTime * eta;
    if (v.type == VehicleType::Ambulance) {
        s += p.wTriage * v.triageLevel * sev;
        if (v.paramedic) s += p.wParamedic * sev;
    } else {
        const double water = std::min(1.0, v.waterLiters / p.referenceWaterLiters);
        if (inc.type == IncidentType::Fire) s += p.wWater * water * sev;
        if (v.ladder && (inc.type == IncidentType::Rescue || inc.severity >= 4)) s += p.wLadder;
    }
    if (v.state == VehicleState::ReturningToBase) s -= p.returningPenalty;
    return s;
}

double Dispatcher::effectivePriority(const Incident& inc) const {
    return inc.severity + w_.policy.agingPerMinute * (w_.now - inc.createdAt);
}

std::vector<Candidate> Dispatcher::rank(const Incident& inc, VehicleType role,
                                        const std::set<VehicleId>& exclude,
                                        bool includePreemptable) const {
    const DispatchPolicy& p = w_.policy;
    std::vector<Candidate> out;
    for (const Vehicle& v : w_.fleet) {
        if (v.type != role || exclude.count(v.id)) continue;

        const bool free = v.state == VehicleState::IdleStation ||
                          (v.state == VehicleState::ReturningToBase && p.interceptReturning);
        bool preemptable = false;
        if (!free && includePreemptable && v.incident >= 0 &&
            (v.state == VehicleState::EnRouteIncident ||
             v.state == VehicleState::StagedAtPerimeter)) {
            preemptable = w_.incident(v.incident).severity <= p.preemptMaxSeverity;
        }
        if (!free && !preemptable) continue;

        const NodeId anchor = anchorNode(v);
        const double delay = anchorDelay(v);
        Candidate c;
        c.vehicle = v.id;
        c.preempt = !free;
        const Route direct = shortestPath(w_.net, anchor, inc.location);
        if (direct.reachable) {
            c.route = direct;
            c.eta = delay + direct.minutes;
        } else {
            const StagingPlan sp = findStagingNode(w_.net, anchor, inc.location);
            if (!sp.found) continue;
            c.route = sp.route;
            c.viaStaging = true;
            c.stagingNode = sp.node;
            c.eta = delay + sp.route.minutes + p.unreachablePenaltyMinutes;
        }
        c.score = suitability(v, inc, c.eta);
        out.push_back(std::move(c));
    }
    std::sort(out.begin(), out.end(), [](const Candidate& a, const Candidate& b) {
        if (a.score != b.score) return a.score > b.score;
        if (a.eta != b.eta) return a.eta < b.eta;
        return a.vehicle < b.vehicle;
    });
    return out;
}

std::vector<Candidate> Dispatcher::previewRanking(IncidentType type, int severity,
                                                  NodeId location, VehicleType role) const {
    Incident probe;
    probe.type = type;
    probe.severity = severity;
    probe.location = location;
    probe.createdAt = w_.now;
    return rank(probe, role, {}, false);
}

IncidentId Dispatcher::submit(IncidentType type, int severity, NodeId location,
                              const std::string& description) {
    if (severity < 1 || severity > 5) throw std::invalid_argument("severity must be 1..5");
    w_.net.node(location);  // bounds check

    Incident inc;
    inc.id = static_cast<IncidentId>(w_.incidents.size());
    inc.type = type;
    inc.severity = severity;
    inc.location = location;
    inc.description = description;
    inc.createdAt = w_.now;
    w_.incidents.push_back(inc);
    w_.stats.incidentsCreated++;
    Incident& ref = w_.incidents.back();

    char buf[200];
    std::snprintf(buf, sizeof buf, "INC#%d created: %s sev %d at %s", ref.id, toString(type),
                  severity, w_.nodeName(location).c_str());
    w_.log(buf);

    if (!tryDispatch(ref, true)) {
        enqueue(ref);
        std::snprintf(buf, sizeof buf, "INC#%d queued (no complete unit package available)", ref.id);
        w_.log(buf);
    }
    return ref.id;
}

void Dispatcher::enqueue(Incident& inc) {
    inc.status = IncidentStatus::Queued;
    inc.queuedSince = w_.now;
    if (std::find(w_.queue.begin(), w_.queue.end(), inc.id) == w_.queue.end())
        w_.queue.push_back(inc.id);
    w_.stats.maxQueueDepth = std::max(w_.stats.maxQueueDepth, static_cast<int>(w_.queue.size()));
}

bool Dispatcher::tryDispatch(Incident& inc, bool allowPreempt) {
    const std::vector<VehicleType> roles = requiredRoles(inc.type, inc.severity);
    const bool mayPreempt = allowPreempt && inc.severity >= w_.policy.preemptMinSeverity;

    // Phase 1: build the whole package without touching any state (atomicity).
    std::set<VehicleId> picked;
    std::vector<Candidate> chosen;
    for (VehicleType role : roles) {
        const std::vector<Candidate> cands = rank(inc, role, picked, mayPreempt);
        if (cands.empty()) return false;  // package cannot be completed -> nothing is assigned
        auto freeIt = std::find_if(cands.begin(), cands.end(),
                                   [](const Candidate& c) { return !c.preempt; });
        const Candidate& best = freeIt != cands.end() ? *freeIt : cands.front();
        picked.insert(best.vehicle);
        chosen.push_back(best);
    }

    // Phase 2: commit.
    if (inc.queuedSince >= 0.0) {
        w_.stats.totalQueueWaitMinutes += w_.now - inc.queuedSince;
        w_.stats.queueWaitSamples++;
        inc.queuedSince = -1.0;
    }
    w_.queue.erase(std::remove(w_.queue.begin(), w_.queue.end(), inc.id), w_.queue.end());
    for (const Candidate& c : chosen)
        if (c.preempt) releaseForPreemption(w_.vehicle(c.vehicle), inc);
    for (const Candidate& c : chosen) assign(inc, c);

    inc.status = IncidentStatus::Dispatched;
    if (inc.firstDispatchAt < 0.0) inc.firstDispatchAt = w_.now;
    if (roles.size() > 1) {
        w_.stats.coDispatches++;
        char buf[120];
        std::snprintf(buf, sizeof buf, "INC#%d co-dispatch package committed atomically (%zu units)",
                      inc.id, roles.size());
        w_.log(buf);
    }
    return true;
}

void Dispatcher::releaseForPreemption(Vehicle& v, const Incident& by) {
    Incident& victim = w_.incident(v.incident);
    victim.assigned.erase(std::remove(victim.assigned.begin(), victim.assigned.end(), v.id),
                          victim.assigned.end());
    victim.preemptedCount++;
    w_.stats.preemptions++;
    char buf[200];
    std::snprintf(buf, sizeof buf, "PREEMPTION: %s reclaimed from INC#%d (sev %d) for INC#%d (sev %d)",
                  v.callsign.c_str(), victim.id, victim.severity, by.id, by.severity);
    w_.log(buf);
    if (victim.assigned.empty()) {
        enqueue(victim);
        std::snprintf(buf, sizeof buf, "INC#%d returned to the queue", victim.id);
        w_.log(buf);
    }
}

void Dispatcher::assign(Incident& inc, const Candidate& c) {
    Vehicle& v = w_.vehicle(c.vehicle);
    v.state = VehicleState::EnRouteIncident;
    v.incident = inc.id;
    v.holding = false;
    v.headingToStaging = c.viaStaging;
    v.destination = c.viaStaging ? c.stagingNode : inc.location;
    v.speedFactor = inc.severity >= w_.policy.signalPreemptionMinSeverity
                        ? w_.policy.signalPreemptionFactor
                        : 1.0;
    setRoute(w_, v, c.route);
    inc.assigned.push_back(v.id);
    v.dispatches++;
    w_.stats.dispatches++;

    char buf[260];
    if (c.viaStaging)
        std::snprintf(buf, sizeof buf,
                      "%s -> INC#%d: incident unreachable, staging at %s (ranking ETA %.1f min incl. cut-off penalty, score %.2f)",
                      v.callsign.c_str(), inc.id, w_.nodeName(c.stagingNode).c_str(), c.eta, c.score);
    else
        std::snprintf(buf, sizeof buf, "%s -> INC#%d at %s (ETA %.1f min, score %.2f)",
                      v.callsign.c_str(), inc.id, w_.nodeName(inc.location).c_str(), c.eta, c.score);
    w_.log(buf);
}

void Dispatcher::processQueue() {
    if (w_.queue.empty()) return;
    std::vector<IncidentId> order = w_.queue;
    std::stable_sort(order.begin(), order.end(), [this](IncidentId a, IncidentId b) {
        const double pa = effectivePriority(w_.incident(a));
        const double pb = effectivePriority(w_.incident(b));
        if (pa != pb) return pa > pb;
        return a < b;
    });

    // Starvation guard: once a higher-priority incident fails to get its units,
    // lower-priority incidents may not grab the roles it is still waiting for.
    std::set<VehicleType> reserved;
    for (IncidentId id : order) {
        Incident& inc = w_.incident(id);
        const std::vector<VehicleType> roles = requiredRoles(inc.type, inc.severity);
        const bool blocked = std::any_of(roles.begin(), roles.end(),
                                         [&](VehicleType r) { return reserved.count(r) > 0; });
        if (!blocked && tryDispatch(inc, true)) continue;
        reserved.insert(roles.begin(), roles.end());
    }
}

}  // namespace v2x

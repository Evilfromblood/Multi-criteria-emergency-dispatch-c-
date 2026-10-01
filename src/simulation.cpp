#include "simulation.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace v2x {

using VS = VehicleState;

Simulation::Simulation(CityData city, DispatchPolicy policy)
    : initial_(std::move(city)), dispatcher_(world_) {
    world_.policy = policy;
    load(initial_);
}

void Simulation::load(const CityData& city) {
    world_.net = city.net;
    world_.fleet = city.fleet;
    for (std::size_t i = 0; i < world_.fleet.size(); ++i) {
        Vehicle& v = world_.fleet[i];
        v.id = static_cast<VehicleId>(i);
        v.node = v.base;
        v.state = VS::IdleStation;
        world_.net.node(v.base);  // validate base
    }
}

void Simulation::reset() {
    std::ostream* echo = world_.echo;
    const DispatchPolicy policy = world_.policy;
    world_ = World();
    world_.echo = echo;
    world_.policy = policy;
    schedule_.clear();
    load(initial_);
}

IncidentId Simulation::createIncident(IncidentType type, int severity, NodeId location,
                                      const std::string& description) {
    const IncidentId id = dispatcher_.submit(type, severity, location, description);
    settle();
    return id;
}

HazardId Simulation::injectHazard(EdgeId edge, HazardKind kind, double multiplier,
                                  const std::string& description) {
    const HazardId id = world_.net.injectHazard(edge, kind, multiplier, description);
    const Edge& e = world_.net.edge(edge);
    char buf[240];
    if (kind == HazardKind::Closure)
        std::snprintf(buf, sizeof buf, "V2X HAZARD#%d: road %s <-> %s CLOSED%s%s", id,
                      world_.nodeName(e.a).c_str(), world_.nodeName(e.b).c_str(),
                      description.empty() ? "" : " - ", description.c_str());
    else
        std::snprintf(buf, sizeof buf, "V2X HAZARD#%d: road %s <-> %s congestion x%.1f%s%s", id,
                      world_.nodeName(e.a).c_str(), world_.nodeName(e.b).c_str(), multiplier,
                      description.empty() ? "" : " - ", description.c_str());
    world_.log(buf);
    rescaleForEdge(edge);
    afterEvents();
    return id;
}

bool Simulation::resolveHazard(HazardId id) {
    if (id < 0 || static_cast<std::size_t>(id) >= world_.net.hazards().size()) return false;
    const EdgeId edge = world_.net.hazards()[static_cast<std::size_t>(id)].edge;
    if (!world_.net.resolveHazard(id)) return false;
    char buf[120];
    std::snprintf(buf, sizeof buf, "V2X HAZARD#%d cleared", id);
    world_.log(buf);
    rescaleForEdge(edge);
    afterEvents();
    return true;
}

void Simulation::rescaleForEdge(EdgeId edgeId) {
    const Edge& e = world_.net.edge(edgeId);
    for (Vehicle& v : world_.fleet) {
        if (!v.midHop || v.hopEdge != edgeId) continue;
        if (e.blocked) continue;  // already committed to this road: keep the old pace
        const double fraction = v.hopRate > 0.0 ? v.hopRemaining / v.hopRate : 0.0;
        v.hopRate = e.travelMinutes() * v.speedFactor;
        v.hopRemaining = fraction * v.hopRate;
    }
}

void Simulation::schedule(double atTime, const std::string& label,
                          std::function<void(Simulation&)> action) {
    schedule_.emplace(atTime, Scheduled{label, std::move(action)});
}

// ---------------------------------------------------------------- time advance

double Simulation::nextEventDelay() const {
    double best = kInf;
    for (const Vehicle& v : world_.fleet) {
        if (v.midHop) best = std::min(best, v.hopRemaining);
        else if (hasTimer(v.state)) best = std::min(best, v.timer);
    }
    if (!schedule_.empty()) best = std::min(best, std::max(0.0, schedule_.begin()->first - world_.now));
    return best;
}

void Simulation::advance(double minutes) {
    if (minutes < 0.0) throw std::invalid_argument("advance: negative time");
    const double target = world_.now + minutes;
    afterEvents();
    long guard = 0;
    while (world_.now < target - kEps) {
        if (++guard > 1000000) throw std::runtime_error("simulation event loop did not converge");
        const double delta = std::max(0.0, std::min(target - world_.now, nextEventDelay()));

        for (Vehicle& v : world_.fleet) {
            if (v.state != VS::IdleStation) v.busyMinutes += delta;
            if (v.midHop) v.hopRemaining -= delta;
            else if (hasTimer(v.state)) v.timer -= delta;
        }
        world_.now += delta;

        for (Vehicle& v : world_.fleet) {
            if (v.midHop && v.hopRemaining <= kEps) completeHop(v);
            else if (hasTimer(v.state) && v.timer <= kEps) completeTimer(v);
        }
        while (!schedule_.empty() && schedule_.begin()->first <= world_.now + kEps) {
            Scheduled s = std::move(schedule_.begin()->second);
            schedule_.erase(schedule_.begin());
            s.action(*this);
        }
        afterEvents();
    }
}

bool Simulation::allQuiet() const {
    if (!world_.queue.empty() || !schedule_.empty()) return false;
    for (const Incident& i : world_.incidents)
        if (i.status != IncidentStatus::Resolved) return false;
    for (const Vehicle& v : world_.fleet)
        if (v.state != VS::IdleStation) return false;
    return true;
}

bool Simulation::runUntilQuiet(double maxMinutes) {
    const double limit = world_.now + maxMinutes;
    while (!allQuiet() && world_.now < limit - kEps) {
        const double step = std::min(1.0, limit - world_.now);
        advance(step);
    }
    return allQuiet();
}

// ---------------------------------------------------------------- state machine

void Simulation::afterEvents() {
    settle();
    retryBlocked();
    dispatcher_.processQueue();
    settle();
}

void Simulation::settle() {
    bool changed = true;
    int guard = 0;
    while (changed && ++guard < 10000) {
        changed = false;
        for (Vehicle& v : world_.fleet) {
            if (isMovingState(v.state) && !v.midHop && v.plan.empty() && !v.holding) {
                onNodeReached(v);
                changed = true;
            }
        }
    }
}

void Simulation::retryBlocked() {
    for (Vehicle& v : world_.fleet) {
        if (v.holding) {
            v.holding = false;
            onNodeReached(v);  // re-plans; sets holding again if still cut off
        } else if (v.state == VS::StagedAtPerimeter) {
            const Incident& inc = world_.incident(v.incident);
            if (shortestPath(world_.net, v.node, inc.location).reachable) {
                v.state = VS::EnRouteIncident;
                world_.stats.resumes++;
                char buf[160];
                std::snprintf(buf, sizeof buf, "%s corridor to %s cleared: resuming transit to INC#%d",
                              v.callsign.c_str(), world_.nodeName(inc.location).c_str(), inc.id);
                world_.log(buf);
                replan(v);
            }
        }
    }
    settle();
}

void Simulation::completeHop(Vehicle& v) {
    v.distanceKm += world_.net.edge(v.hopEdge).distanceKm;
    v.node = v.nextNode;
    v.midHop = false;
    v.hopEdge = -1;
    v.nextNode = -1;
    v.hopRemaining = 0.0;
    onNodeReached(v);
}

void Simulation::completeTimer(Vehicle& v) {
    v.timer = 0.0;
    if (v.state == VS::OnScene) {
        completeScene(v);
    } else if (v.state == VS::AtHospitalTurnover) {
        v.state = VS::ReturningToBase;
        v.speedFactor = 1.0;
        char buf[160];
        std::snprintf(buf, sizeof buf, "%s turnover complete at %s, returning to base",
                      v.callsign.c_str(), world_.nodeName(v.node).c_str());
        world_.log(buf);
        if (!replan(v)) v.holding = true;
    }
}

void Simulation::onNodeReached(Vehicle& v) {
    char buf[200];
    switch (v.state) {
        case VS::EnRouteIncident: {
            const Incident& inc = world_.incident(v.incident);
            if (v.node == inc.location) {
                arriveOnScene(v);
                return;
            }
            if (!replan(v)) {
                v.holding = true;
                return;
            }
            if (!v.midHop && v.plan.empty()) {  // incident unreachable and we are already at the staging node
                v.state = VS::StagedAtPerimeter;
                v.holding = false;
                world_.stats.stagings++;
                std::snprintf(buf, sizeof buf, "%s STAGED at %s: INC#%d at %s is cut off",
                              v.callsign.c_str(), world_.nodeName(v.node).c_str(), inc.id,
                              world_.nodeName(inc.location).c_str());
                world_.log(buf);
            }
            return;
        }
        case VS::TransportingHospital: {
            if (world_.net.node(v.node).kind == NodeKind::Hospital) {
                v.state = VS::AtHospitalTurnover;
                v.timer = world_.policy.hospitalTurnoverMinutes;
                v.plan.clear();
                v.holding = false;
                std::snprintf(buf, sizeof buf, "%s arrived at %s (turnover %.0f min)",
                              v.callsign.c_str(), world_.nodeName(v.node).c_str(), v.timer);
                world_.log(buf);
                return;
            }
            if (!replan(v)) v.holding = true;
            return;
        }
        case VS::ReturningToBase: {
            if (v.node == v.base) {
                v.state = VS::IdleStation;
                v.plan.clear();
                v.destination = -1;
                v.holding = false;
                v.speedFactor = 1.0;
                std::snprintf(buf, sizeof buf, "%s back at base (%s), available",
                              v.callsign.c_str(), world_.nodeName(v.base).c_str());
                world_.log(buf);
                return;
            }
            if (!replan(v)) v.holding = true;
            return;
        }
        default:
            return;
    }
}

bool Simulation::replan(Vehicle& v) {
    const NodeId anchor = anchorNode(v);
    Route route;
    switch (v.state) {
        case VS::EnRouteIncident: {
            const Incident& inc = world_.incident(v.incident);
            const Route direct = shortestPath(world_.net, anchor, inc.location);
            if (direct.reachable) {
                v.destination = inc.location;
                v.headingToStaging = false;
                route = direct;
            } else {
                const StagingPlan sp = findStagingNode(world_.net, anchor, inc.location);
                if (!sp.found) return false;
                v.destination = sp.node;
                v.headingToStaging = true;
                route = sp.route;
            }
            break;
        }
        case VS::TransportingHospital: {
            const NodeId h = chooseHospital(anchor);
            if (h < 0) return false;
            v.destination = h;
            route = shortestPath(world_.net, anchor, h);
            break;
        }
        case VS::ReturningToBase: {
            route = shortestPath(world_.net, anchor, v.base);
            if (!route.reachable) return false;
            v.destination = v.base;
            break;
        }
        default:
            return false;
    }
    const std::deque<NodeId> newPlan(route.path.begin() + 1, route.path.end());
    if (!v.plan.empty() && newPlan != v.plan) {
        world_.stats.reroutes++;
        char buf[200];
        std::snprintf(buf, sizeof buf, "%s REROUTED around changed road conditions (new ETA %.1f min)",
                      v.callsign.c_str(), anchorDelay(v) + route.minutes);
        world_.log(buf);
    }
    setRoute(world_, v, route);
    return true;
}

NodeId Simulation::chooseHospital(NodeId from) const {
    const ShortestPathTree tree = dijkstra(world_.net, from);
    NodeId best = -1;
    double bestD = kInf;
    for (NodeId h : world_.net.nodesOfKind(NodeKind::Hospital)) {
        const double d = tree.dist[static_cast<std::size_t>(h)];
        if (d < bestD - kEps) {
            bestD = d;
            best = h;
        }
    }
    return best;
}

double Simulation::sceneMinutes(const Vehicle& v, const Incident& inc) const {
    return v.type == VehicleType::Ambulance ? 8.0 + 2.0 * inc.severity : 10.0 + 4.0 * inc.severity;
}

void Simulation::arriveOnScene(Vehicle& v) {
    Incident& inc = world_.incident(v.incident);
    v.state = VS::OnScene;
    v.holding = false;
    v.headingToStaging = false;
    v.plan.clear();
    v.timer = sceneMinutes(v, inc);
    if (inc.firstArrivalAt < 0.0) {
        inc.firstArrivalAt = world_.now;
        inc.status = IncidentStatus::OnScene;
        world_.stats.totalResponseMinutes += world_.now - inc.createdAt;
        world_.stats.responseSamples++;
    }
    char buf[200];
    std::snprintf(buf, sizeof buf, "%s ON SCENE at INC#%d (%s), working %.0f min", v.callsign.c_str(),
                  inc.id, world_.nodeName(inc.location).c_str(), v.timer);
    world_.log(buf);
}

void Simulation::completeScene(Vehicle& v) {
    Incident& inc = world_.incident(v.incident);
    inc.assigned.erase(std::remove(inc.assigned.begin(), inc.assigned.end(), v.id), inc.assigned.end());
    inc.completed.push_back(v.id);
    const bool transport = v.type == VehicleType::Ambulance &&
                           (inc.type == IncidentType::Medical || inc.severity >= 3);
    v.incident = -1;
    v.speedFactor = 1.0;
    v.holding = false;
    char buf[200];
    std::snprintf(buf, sizeof buf, "%s finished on scene at INC#%d%s", v.callsign.c_str(), inc.id,
                  transport ? ", transporting patient" : ", returning to base");
    world_.log(buf);
    if (inc.assigned.empty()) resolveIncident(inc);
    v.state = transport ? VS::TransportingHospital : VS::ReturningToBase;
    if (!replan(v)) v.holding = true;
}

void Simulation::resolveIncident(Incident& inc) {
    inc.status = IncidentStatus::Resolved;
    inc.resolvedAt = world_.now;
    world_.stats.incidentsResolved++;
    char buf[100];
    std::snprintf(buf, sizeof buf, "INC#%d RESOLVED", inc.id);
    world_.log(buf);
}

}  // namespace v2x

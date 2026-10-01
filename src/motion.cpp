#include "motion.hpp"

#include <stdexcept>

namespace v2x {

namespace {
void startHop(World& w, Vehicle& v) {
    const NodeId next = v.plan.front();
    v.plan.pop_front();
    const auto e = w.net.edgeBetween(v.node, next);
    if (!e) throw std::logic_error("startHop: route uses a non-existent road");
    double minutes = w.net.edge(*e).travelMinutes() * v.speedFactor;
    if (minutes == kInf)  // road closed after the route was planned: stay committed
        minutes = w.net.edge(*e).distanceKm / w.net.edge(*e).speedKmh * 60.0;
    v.nextNode = next;
    v.hopEdge = *e;
    v.hopRate = minutes;
    v.hopRemaining = minutes;
    v.midHop = true;
}
}  // namespace

void setRoute(World& w, Vehicle& v, const Route& route) {
    if (!route.reachable || route.path.empty() || route.path.front() != anchorNode(v))
        throw std::logic_error("setRoute: route does not start at the vehicle anchor");
    v.plan.assign(route.path.begin() + 1, route.path.end());
    if (!v.midHop && !v.plan.empty()) startHop(w, v);
}

double remainingTravelMinutes(const World& w, const Vehicle& v) {
    if (!v.midHop) return v.plan.empty() ? 0.0 : -1.0;
    double total = v.hopRemaining;
    NodeId prev = v.nextNode;
    for (NodeId n : v.plan) {
        const auto e = w.net.edgeBetween(prev, n);
        if (!e) return -1.0;
        const double t = w.net.edge(*e).travelMinutes() * v.speedFactor;
        if (t == kInf) return -1.0;
        total += t;
        prev = n;
    }
    return total;
}

}  // namespace v2x

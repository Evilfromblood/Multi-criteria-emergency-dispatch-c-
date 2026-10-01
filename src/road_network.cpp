#include "road_network.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace v2x {

NodeId RoadNetwork::addNode(const std::string& name, double x, double y, NodeKind kind) {
    Node n;
    n.id = static_cast<NodeId>(nodes_.size());
    n.name = name;
    n.x = x;
    n.y = y;
    n.kind = kind;
    nodes_.push_back(n);
    adj_.emplace_back();
    return n.id;
}

EdgeId RoadNetwork::addEdge(NodeId a, NodeId b, double distanceKm, double speedKmh) {
    node(a);  // bounds checks (throw std::out_of_range)
    node(b);
    if (a == b) throw std::invalid_argument("addEdge: self loops are not allowed");
    if (distanceKm <= 0.0 || speedKmh <= 0.0)
        throw std::invalid_argument("addEdge: distance and speed must be positive");
    Edge e;
    e.id = static_cast<EdgeId>(edges_.size());
    e.a = a;
    e.b = b;
    e.distanceKm = distanceKm;
    e.speedKmh = speedKmh;
    edges_.push_back(e);
    adj_[static_cast<std::size_t>(a)].push_back({b, e.id});
    adj_[static_cast<std::size_t>(b)].push_back({a, e.id});
    return e.id;
}

const Node& RoadNetwork::node(NodeId id) const {
    return nodes_.at(static_cast<std::size_t>(id));
}
const Edge& RoadNetwork::edge(EdgeId id) const {
    return edges_.at(static_cast<std::size_t>(id));
}
const std::vector<Neighbor>& RoadNetwork::neighbors(NodeId id) const {
    return adj_.at(static_cast<std::size_t>(id));
}

std::optional<EdgeId> RoadNetwork::edgeBetween(NodeId a, NodeId b) const {
    for (const Neighbor& nb : neighbors(a))
        if (nb.to == b) return nb.edge;
    return std::nullopt;
}

std::vector<NodeId> RoadNetwork::nodesOfKind(NodeKind kind) const {
    std::vector<NodeId> out;
    for (const Node& n : nodes_)
        if (n.kind == kind) out.push_back(n.id);
    return out;
}

double RoadNetwork::euclideanKm(NodeId a, NodeId b) const {
    const Node& na = node(a);
    const Node& nb = node(b);
    return std::hypot(na.x - nb.x, na.y - nb.y);
}

HazardId RoadNetwork::injectHazard(EdgeId edgeId, HazardKind kind, double multiplier,
                                   const std::string& description) {
    edge(edgeId);  // bounds check
    if (kind == HazardKind::Congestion && multiplier < 1.0)
        throw std::invalid_argument("congestion multiplier must be >= 1.0");
    Hazard h;
    h.id = static_cast<HazardId>(hazards_.size());
    h.edge = edgeId;
    h.kind = kind;
    h.multiplier = kind == HazardKind::Closure ? 1.0 : multiplier;
    h.description = description;
    h.active = true;
    hazards_.push_back(h);
    recomputeEdge(edgeId);
    return h.id;
}

bool RoadNetwork::resolveHazard(HazardId id) {
    if (id < 0 || static_cast<std::size_t>(id) >= hazards_.size()) return false;
    Hazard& h = hazards_[static_cast<std::size_t>(id)];
    if (!h.active) return false;
    h.active = false;
    recomputeEdge(h.edge);
    return true;
}

int RoadNetwork::activeHazardCount() const {
    return static_cast<int>(
        std::count_if(hazards_.begin(), hazards_.end(), [](const Hazard& h) { return h.active; }));
}

void RoadNetwork::recomputeEdge(EdgeId id) {
    Edge& e = edges_[static_cast<std::size_t>(id)];
    e.congestion = 1.0;
    e.blocked = false;
    for (const Hazard& h : hazards_) {
        if (!h.active || h.edge != id) continue;
        if (h.kind == HazardKind::Closure) e.blocked = true;
        else e.congestion = std::max(e.congestion, h.multiplier);
    }
}

}  // namespace v2x

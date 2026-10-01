#pragma once
#include <optional>
#include <string>
#include <vector>

#include "types.hpp"

namespace v2x {

struct Node {
    NodeId id = 0;
    std::string name;
    double x = 0.0;  // km, used for geometric "closeness" (perimeter staging)
    double y = 0.0;
    NodeKind kind = NodeKind::Junction;
};

struct Edge {
    EdgeId id = 0;
    NodeId a = 0;
    NodeId b = 0;
    double distanceKm = 1.0;
    double speedKmh = 50.0;
    double congestion = 1.0;  // multiplier >= 1, driven by V2X hazards
    bool blocked = false;     // driven by V2X closures

    // travel_time_minutes = (distance_km / speed_kmh) * congestion_multiplier * 60
    double travelMinutes() const {
        return blocked ? kInf : distanceKm / speedKmh * congestion * 60.0;
    }
    NodeId other(NodeId n) const { return n == a ? b : a; }
};

struct Hazard {
    HazardId id = 0;
    EdgeId edge = 0;
    HazardKind kind = HazardKind::Congestion;
    double multiplier = 1.0;
    std::string description;
    bool active = true;
};

struct Neighbor {
    NodeId to;
    EdgeId edge;
};

class RoadNetwork {
public:
    NodeId addNode(const std::string& name, double x, double y, NodeKind kind);
    EdgeId addEdge(NodeId a, NodeId b, double distanceKm, double speedKmh);

    const Node& node(NodeId id) const;
    const Edge& edge(EdgeId id) const;
    std::size_t nodeCount() const { return nodes_.size(); }
    std::size_t edgeCount() const { return edges_.size(); }
    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<Edge>& edges() const { return edges_; }
    const std::vector<Neighbor>& neighbors(NodeId id) const;
    std::optional<EdgeId> edgeBetween(NodeId a, NodeId b) const;
    std::vector<NodeId> nodesOfKind(NodeKind kind) const;
    double euclideanKm(NodeId a, NodeId b) const;

    // V2X hazard broadcasts. Several hazards may overlap on one edge:
    // the worst congestion multiplier wins, any active closure blocks the edge.
    HazardId injectHazard(EdgeId edge, HazardKind kind, double multiplier,
                          const std::string& description);
    bool resolveHazard(HazardId id);
    const std::vector<Hazard>& hazards() const { return hazards_; }
    int activeHazardCount() const;

private:
    void recomputeEdge(EdgeId id);

    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    std::vector<std::vector<Neighbor>> adj_;
    std::vector<Hazard> hazards_;
};

}  // namespace v2x

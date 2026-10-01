#pragma once
#include <vector>

#include "road_network.hpp"

namespace v2x {

struct Route {
    bool reachable = false;
    std::vector<NodeId> path;  // includes both endpoints
    double minutes = kInf;
};

struct ShortestPathTree {
    std::vector<double> dist;
    std::vector<NodeId> prev;
};

// Dijkstra over current (dynamic) edge travel times. Blocked edges are skipped.
ShortestPathTree dijkstra(const RoadNetwork& net, NodeId source);
Route extractRoute(const ShortestPathTree& tree, NodeId source, NodeId target);
Route shortestPath(const RoadNetwork& net, NodeId from, NodeId to);

// Policy coefficients for picking a perimeter staging node (simulation heuristic).
struct StagingWeights {
    double gapKm = 1.0;     // straight-line distance from staging node to incident
    double minutes = 0.01;  // travel time needed to reach the staging node
};

struct StagingPlan {
    bool found = false;
    NodeId node = -1;
    Route route;
    double gapKm = kInf;
};

// When `incident` is unreachable from `from`, choose the reachable node that
// is geometrically closest to the incident (travel time is a tie-breaker).
StagingPlan findStagingNode(const RoadNetwork& net, NodeId from, NodeId incident,
                            const StagingWeights& weights = StagingWeights());

}  // namespace v2x

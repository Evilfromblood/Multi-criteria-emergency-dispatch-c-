#include "router.hpp"

#include <algorithm>
#include <functional>
#include <queue>
#include <utility>

namespace v2x {

ShortestPathTree dijkstra(const RoadNetwork& net, NodeId source) {
    net.node(source);  // bounds check
    const std::size_t n = net.nodeCount();
    ShortestPathTree t;
    t.dist.assign(n, kInf);
    t.prev.assign(n, -1);

    using QItem = std::pair<double, NodeId>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    t.dist[static_cast<std::size_t>(source)] = 0.0;
    pq.push({0.0, source});

    while (!pq.empty()) {
        const QItem top = pq.top();
        pq.pop();
        const double d = top.first;
        const NodeId u = top.second;
        if (d > t.dist[static_cast<std::size_t>(u)]) continue;
        for (const Neighbor& nb : net.neighbors(u)) {
            const double w = net.edge(nb.edge).travelMinutes();
            if (w == kInf) continue;  // closed road
            const double nd = d + w;
            const std::size_t v = static_cast<std::size_t>(nb.to);
            if (nd < t.dist[v] - kEps) {
                t.dist[v] = nd;
                t.prev[v] = u;
                pq.push({nd, nb.to});
            }
        }
    }
    return t;
}

Route extractRoute(const ShortestPathTree& tree, NodeId source, NodeId target) {
    Route r;
    const double d = tree.dist.at(static_cast<std::size_t>(target));
    if (d == kInf) return r;
    r.reachable = true;
    r.minutes = d;
    for (NodeId cur = target; cur != -1; cur = tree.prev[static_cast<std::size_t>(cur)]) {
        r.path.push_back(cur);
        if (cur == source) break;
    }
    std::reverse(r.path.begin(), r.path.end());
    return r;
}

Route shortestPath(const RoadNetwork& net, NodeId from, NodeId to) {
    net.node(to);  // bounds check
    return extractRoute(dijkstra(net, from), from, to);
}

Route aStar(const RoadNetwork& net, NodeId from, NodeId to) {
    net.node(from);  // bounds checks
    net.node(to);
    if (from == to) {
        Route r;
        r.reachable = true;
        r.path = {from};
        r.minutes = 0.0;
        return r;
    }

    const std::size_t n = net.nodeCount();
    const double maxSpd = net.maxSpeedKmh();
    auto heuristic = [&](NodeId u) -> double {
        return (net.euclideanKm(u, to) / maxSpd) * 60.0;
    };

    std::vector<double> gScore(n, kInf);
    std::vector<NodeId> prev(n, -1);
    gScore[static_cast<std::size_t>(from)] = 0.0;

    using QItem = std::pair<double, NodeId>;  // {fScore, u}
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    pq.push({heuristic(from), from});

    while (!pq.empty()) {
        const QItem top = pq.top();
        pq.pop();
        const double f = top.first;
        const NodeId u = top.second;
        if (u == to) break;

        const double g_u = gScore[static_cast<std::size_t>(u)];
        if (f > g_u + heuristic(u) + kEps) continue;

        for (const Neighbor& nb : net.neighbors(u)) {
            const double w = net.edge(nb.edge).travelMinutes();
            if (w == kInf) continue;  // closed road
            const double ng = g_u + w;
            const std::size_t v = static_cast<std::size_t>(nb.to);
            if (ng < gScore[v] - kEps) {
                gScore[v] = ng;
                prev[v] = u;
                const double nf = ng + heuristic(nb.to);
                pq.push({nf, nb.to});
            }
        }
    }

    Route r;
    const double total = gScore[static_cast<std::size_t>(to)];
    if (total == kInf) return r;
    r.reachable = true;
    r.minutes = total;
    for (NodeId cur = to; cur != -1; cur = prev[static_cast<std::size_t>(cur)]) {
        r.path.push_back(cur);
        if (cur == from) break;
    }
    std::reverse(r.path.begin(), r.path.end());
    return r;
}

StagingPlan findStagingNode(const RoadNetwork& net, NodeId from, NodeId incident,
                            const StagingWeights& weights) {
    const ShortestPathTree tree = dijkstra(net, from);
    StagingPlan best;
    double bestScore = kInf;
    for (const Node& n : net.nodes()) {
        const double d = tree.dist[static_cast<std::size_t>(n.id)];
        if (d == kInf || n.id == incident) continue;
        const double gap = net.euclideanKm(n.id, incident);
        const double score = weights.gapKm * gap + weights.minutes * d;
        if (score < bestScore - kEps) {
            bestScore = score;
            best.found = true;
            best.node = n.id;
            best.gapKm = gap;
        }
    }
    if (best.found) best.route = extractRoute(tree, from, best.node);
    return best;
}

}  // namespace v2x

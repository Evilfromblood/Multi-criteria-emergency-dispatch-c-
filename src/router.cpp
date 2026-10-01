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

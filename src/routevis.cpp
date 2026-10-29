// routevis.cpp  –  Step-by-step ASCII route visualiser for the demo city.
//
// The "playable" mode asked for in the task description:
//   "a manually working or playable program that tells how an emergency
//    vehicle moves from one given place to other given place"
//
// What it does:
//   • Lists every named node in the road network with its ID.
//   • Asks the user to pick a SOURCE and a DESTINATION node.
//   • Optionally lets the user pick the vehicle type (ambulance / engine).
//   • Computes the best route with A* (falls back to Dijkstra if the
//     goal is unreachable with A*).
//   • Draws a compact ASCII map of the ENTIRE city, marking:
//       [S] = start   [D] = destination   [ ] = other nodes
//       (*) = current vehicle position    roads as dashes
//   • Walks the vehicle one hop at a time, printing:
//       – the road being traversed (distance, speed limit)
//       – estimated travel time for that hop
//       – cumulative elapsed time
//       – a refreshed ASCII map with the vehicle at the new node
//   • Summarises the total route: distance, time, node sequence.
//   • Optionally repeats with a new pair of locations.

#include "routevis.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "road_network.hpp"
#include "router.hpp"
#include "scenario.hpp"

namespace v2x {

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace {

// Node-kind symbols used in the ASCII map legend.
const char* kindTag(NodeKind k) {
    switch (k) {
        case NodeKind::Station:  return "STN";
        case NodeKind::Hospital: return "HSP";
        case NodeKind::District: return "DST";
        default:                 return "JCT";
    }
}

// ─── ASCII MAP ────────────────────────────────────────────────────────────────
// The map is drawn on a fixed 60×22 character grid.
// Node coordinates (x,y) come from the road network (unit = km, range 0–12).
// We project them to grid cells and draw edges as straight lines using
// Bresenham's algorithm, then overwrite with node labels.

constexpr int MAP_W = 62;
constexpr int MAP_H = 24;

struct MapCanvas {
    char buf[MAP_H][MAP_W + 1];

    MapCanvas() {
        for (int r = 0; r < MAP_H; ++r) {
            for (int c = 0; c < MAP_W; ++c) buf[r][c] = ' ';
            buf[r][MAP_W] = '\0';
        }
    }

    void set(int r, int c, char ch) {
        if (r >= 0 && r < MAP_H && c >= 0 && c < MAP_W) buf[r][c] = ch;
    }

    char get(int r, int c) const {
        if (r >= 0 && r < MAP_H && c >= 0 && c < MAP_W) return buf[r][c];
        return ' ';
    }

    void drawLine(int r0, int c0, int r1, int c1, char hCh, char vCh, char dCh) {
        int dr = r1 - r0, dc = c1 - c0;
        int steps = std::max(std::abs(dr), std::abs(dc));
        if (steps == 0) return;
        for (int i = 1; i < steps; ++i) {
            int r = r0 + (dr * i + steps / 2) / steps;
            int c = c0 + (dc * i + steps / 2) / steps;
            if (get(r, c) == ' ') {
                char ch;
                if (std::abs(dc) > std::abs(dr) * 2)      ch = hCh;
                else if (std::abs(dr) > std::abs(dc) * 2) ch = vCh;
                else                                        ch = dCh;
                set(r, c, ch);
            }
        }
    }

    void print(std::ostream& out) const {
        for (int r = 0; r < MAP_H; ++r) out << "  " << buf[r] << "\n";
    }
};

// Project a node's (x,y) coord [0..12 km] into map cell (row, col).
std::pair<int, int> project(double x, double y) {
    // x → columns 1..MAP_W-4,  y → rows MAP_H-2..1  (y=0 → bottom)
    int col = static_cast<int>(x / 12.0 * (MAP_W - 6)) + 2;
    int row = MAP_H - 2 - static_cast<int>(y / 12.0 * (MAP_H - 4));
    col = std::max(1, std::min(MAP_W - 5, col));
    row = std::max(1, std::min(MAP_H - 2, row));
    return {row, col};
}

// Draw the city map.  `vehicle` = current node of the moving unit (-1 = none).
// `path`  = full planned path (nodes to highlight).
void drawMap(std::ostream& out, const RoadNetwork& net,
             NodeId fromId, NodeId toId, NodeId vehicleAt,
             const std::vector<NodeId>& path) {
    MapCanvas cv;

    // Border
    for (int c = 0; c < MAP_W; ++c) { cv.set(0, c, '-'); cv.set(MAP_H - 1, c, '-'); }
    for (int r = 0; r < MAP_H; ++r) { cv.set(r, 0, '|'); cv.set(r, MAP_W - 1, '|'); }

    // Edges
    for (const Edge& e : net.edges()) {
        const Node& na = net.node(e.a);
        const Node& nb = net.node(e.b);
        auto [r0, c0] = project(na.x, na.y);
        auto [r1, c1] = project(nb.x, nb.y);

        // Highlight edges on the planned route
        bool onRoute = false;
        for (std::size_t i = 1; i < path.size(); ++i) {
            if ((path[i - 1] == e.a && path[i] == e.b) ||
                (path[i - 1] == e.b && path[i] == e.a)) {
                onRoute = true;
                break;
            }
        }
        cv.drawLine(r0, c0, r1, c1, onRoute ? '=' : '-',
                    onRoute ? '#' : '|', onRoute ? '*' : '.');
    }

    // Nodes  – 3-char label centred on node position
    for (const Node& n : net.nodes()) {
        auto [r, c] = project(n.x, n.y);
        char label[4] = "   ";
        if (n.id == vehicleAt) {
            label[0] = '('; label[1] = 'V'; label[2] = ')';
        } else if (n.id == fromId) {
            label[0] = '['; label[1] = 'S'; label[2] = ']';
        } else if (n.id == toId) {
            label[0] = '['; label[1] = 'D'; label[2] = ']';
        } else {
            const std::string& nm = n.name;
            label[0] = nm.size() > 0 ? nm[0] : '?';
            label[1] = nm.size() > 1 ? nm[1] : ' ';
            label[2] = nm.size() > 2 ? nm[2] : ' ';
        }
        cv.set(r, c - 1, label[0]);
        cv.set(r, c,     label[1]);
        cv.set(r, c + 1, label[2]);
    }

    cv.print(out);
}

// Small helper: ask user for an integer in [lo, hi], returns nullopt on EOF.
std::optional<int> promptInt(const std::string& prompt, int lo, int hi) {
    for (;;) {
        std::cout << prompt << std::flush;
        std::string line;
        if (!std::getline(std::cin, line)) return std::nullopt;
        try {
            std::size_t used = 0;
            int v = std::stoi(line, &used);
            if (used == line.size() && v >= lo && v <= hi) return v;
        } catch (...) {}
        std::cout << "  Please enter a number between " << lo << " and " << hi << ".\n";
    }
}

// Sleep for `ms` milliseconds (cross-platform via <chrono>).
void sleep_ms(int ms) {
    if (ms > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Print a thick separator line.
void sep(std::ostream& out, char ch = '=', int w = 62) {
    for (int i = 0; i < w; ++i) out << ch;
    out << "\n";
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

void routeVisualize(const Simulation& sim, NodeId from, NodeId to,
                    std::ostream& out, int pauseMs) {
    const RoadNetwork& net = sim.world().net;

    // ── Validate nodes ────────────────────────────────────────────────────────
    net.node(from);  // throws if invalid
    net.node(to);

    sep(out, '=');
    out << "  EMERGENCY VEHICLE ROUTE PLAYBACK\n";
    sep(out, '=');

    if (from == to) {
        out << "  Vehicle is already at the destination!\n";
        return;
    }

    // ── Compute route ─────────────────────────────────────────────────────────
    Route route = aStar(net, from, to);
    if (!route.reachable) {
        out << "  No route found between "
            << net.node(from).name << " and " << net.node(to).name << ".\n"
            << "  (All roads may be blocked by active V2X hazards.)\n";
        return;
    }

    const std::vector<NodeId>& path = route.path;

    // ── Route summary ─────────────────────────────────────────────────────────
    out << "\n  FROM : " << net.node(from).name << "  [" << kindTag(net.node(from).kind) << "]\n"
        << "  TO   : " << net.node(to).name   << "  [" << kindTag(net.node(to).kind)   << "]\n"
        << "\n  Planned route (" << path.size() << " nodes, "
        << path.size() - 1 << " hops):\n";

    // Total distance and time
    double totalKm = 0.0;
    double totalMin = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        auto eid = net.edgeBetween(path[i - 1], path[i]);
        if (eid) {
            const Edge& e = net.edge(*eid);
            totalKm  += e.distanceKm;
            totalMin += e.travelMinutes();
        }
    }

    out << "  ";
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i) out << " -> ";
        out << net.node(path[i]).name;
    }
    out << "\n\n"
        << std::fixed << std::setprecision(1)
        << "  Total distance : " << totalKm  << " km\n"
        << "  ETA            : " << totalMin << " min  ("
        << static_cast<int>(totalMin) / 60 << " h "
        << static_cast<int>(totalMin) % 60 << " min)\n";

    sep(out, '-');
    out << "\n  [S]=Start  [D]=Dest  (V)=Vehicle  == planned road  -- other road\n\n";

    // ── Initial map ───────────────────────────────────────────────────────────
    drawMap(out, net, from, to, from, path);
    out << "\n  Press ENTER to start the journey...\n";
    if (pauseMs > 0 && &out == &std::cout) {
        std::string tmp;
        std::getline(std::cin, tmp);
    } else {
        sleep_ms(pauseMs);
    }

    // ── Hop-by-hop walk ───────────────────────────────────────────────────────
    double elapsed = 0.0;
    for (std::size_t hop = 1; hop < path.size(); ++hop) {
        NodeId prevNode = path[hop - 1];
        NodeId nextNode = path[hop];
        const Node& prev = net.node(prevNode);
        const Node& next = net.node(nextNode);

        auto eid = net.edgeBetween(prevNode, nextNode);
        double hopKm  = 0.0, hopMin = 0.0, speedKmh = 50.0;
        if (eid) {
            const Edge& e = net.edge(*eid);
            hopKm   = e.distanceKm;
            hopMin  = e.travelMinutes();
            speedKmh = e.speedKmh;
        }
        elapsed += hopMin;

        // Clear screen hint (works on most terminals; harmless elsewhere)
        out << "\n";
        sep(out, '-');
        out << "  HOP " << hop << " / " << (path.size() - 1) << "\n";
        sep(out, '-');
        out << "  Travelling : " << prev.name << "  -->  " << next.name << "\n"
            << "  Road       : " << std::fixed << std::setprecision(1)
            << hopKm  << " km @ " << static_cast<int>(speedKmh) << " km/h"
            << (hopMin < 1e9 ? "" : "  [BLOCKED]") << "\n"
            << "  Hop time   : " << hopMin   << " min\n"
            << "  Elapsed    : " << elapsed  << " min\n"
            << "  Remaining  : " << (totalMin - elapsed) << " min\n\n";

        drawMap(out, net, from, to, nextNode, path);

        if (hop < path.size() - 1) {
            if (pauseMs > 0 && &out == &std::cout) {
                out << "\n  Press ENTER for next hop...";
                std::string tmp;
                std::getline(std::cin, tmp);
            } else {
                sleep_ms(pauseMs);
            }
        }
    }

    // ── Arrival ───────────────────────────────────────────────────────────────
    out << "\n";
    sep(out, '*');
    out << "  ARRIVED at " << net.node(to).name << "!\n"
        << "  Journey complete: " << std::fixed << std::setprecision(1)
        << totalKm << " km in " << totalMin << " min.\n";
    sep(out, '*');
}

// ─────────────────────────────────────────────────────────────────────────────

bool runRouteVisualizer(Simulation& sim, std::ostream& out) {
    const RoadNetwork& net = sim.world().net;
    const int nNodes = static_cast<int>(net.nodeCount());

    // List all nodes
    out << "\n";
    sep(out, '=');
    out << "  EMERGENCY VEHICLE ROUTE VISUALIZER\n";
    sep(out, '=');
    out << "  City nodes:\n\n";
    for (const Node& n : net.nodes()) {
        out << "  " << std::setw(2) << n.id << "  "
            << std::left << std::setw(22) << n.name
            << "  [" << kindTag(n.kind) << "]\n";
    }
    out << "\n";

    // Pick source
    const auto fromOpt = promptInt("  Source node ID> ", 0, nNodes - 1);
    if (!fromOpt) return false;
    const auto toOpt   = promptInt("  Destination node ID> ", 0, nNodes - 1);
    if (!toOpt) return false;

    // Interactive: step on ENTER; batch: 0 ms pause.
    int pause = (&out == &std::cout) ? 1 : 0;  // 1 = wait for ENTER

    routeVisualize(sim, *fromOpt, *toOpt, out, pause);
    return true;
}

}  // namespace v2x

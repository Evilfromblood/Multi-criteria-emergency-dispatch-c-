// Self-contained test runner (no external framework) - run via CTest or directly.
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "scenario.hpp"
#include "telemetry.hpp"

using namespace v2x;
using namespace v2x::city;

namespace {

int g_checks = 0;
int g_failed = 0;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        ++g_checks;                                                                    \
        if (!(cond)) {                                                                 \
            ++g_failed;                                                                \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << "  " #cond "\n"; \
        }                                                                              \
    } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) <= (eps))

// Demo city, optionally keeping only the units for which `keep` is true.
CityData cityWith(const std::function<bool(const Vehicle&)>& keep) {
    CityData c = buildDemoCity();
    c.fleet.erase(std::remove_if(c.fleet.begin(), c.fleet.end(),
                                 [&](const Vehicle& v) { return !keep(v); }),
                  c.fleet.end());
    return c;
}

CityData oneAmbulanceCity() {
    return cityWith([](const Vehicle& v) { return v.callsign == "A1"; });
}

// ------------------------------------------------------------------ network

void testNetworkConstruction() {
    const CityData c = buildDemoCity();
    CHECK(c.net.nodeCount() == 15);
    CHECK(c.net.edgeCount() == 21);
    CHECK(c.net.nodesOfKind(NodeKind::Hospital).size() == 2);
    CHECK(c.net.nodesOfKind(NodeKind::Station).size() == 3);
    CHECK(c.fleet.size() == 7);
    CHECK(c.net.edgeBetween(Central, NorthHub).has_value());
    CHECK(!c.net.edgeBetween(Central, Hillcrest).has_value());

    RoadNetwork n;
    const NodeId a = n.addNode("a", 0, 0, NodeKind::Junction);
    const NodeId b = n.addNode("b", 1, 0, NodeKind::Junction);
    bool threw = false;
    try { n.addEdge(a, a, 1, 1); } catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { n.addEdge(a, b, -1, 10); } catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { n.addEdge(a, 99, 1, 10); } catch (const std::out_of_range&) { threw = true; }
    CHECK(threw);
}

void testTravelTimeFormula() {
    RoadNetwork n;
    const NodeId a = n.addNode("a", 0, 0, NodeKind::Junction);
    const NodeId b = n.addNode("b", 3, 0, NodeKind::Junction);
    const EdgeId e = n.addEdge(a, b, 3.0, 60.0);
    CHECK_NEAR(n.edge(e).travelMinutes(), 3.0, 1e-9);  // 3 km / 60 km/h * 60
    const HazardId h = n.injectHazard(e, HazardKind::Congestion, 2.0, "");
    CHECK_NEAR(n.edge(e).travelMinutes(), 6.0, 1e-9);
    CHECK(n.resolveHazard(h));
    CHECK_NEAR(n.edge(e).travelMinutes(), 3.0, 1e-9);
    CHECK(!n.resolveHazard(h));  // already cleared
    const HazardId c = n.injectHazard(e, HazardKind::Closure, 1.0, "");
    CHECK(n.edge(e).blocked);
    CHECK(n.edge(e).travelMinutes() == kInf);
    n.resolveHazard(c);
    CHECK(!n.edge(e).blocked);

    bool threw = false;
    try { n.injectHazard(e, HazardKind::Congestion, 0.5, ""); } catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
}

void testOverlappingHazards() {
    RoadNetwork n;
    const NodeId a = n.addNode("a", 0, 0, NodeKind::Junction);
    const NodeId b = n.addNode("b", 1, 0, NodeKind::Junction);
    const EdgeId e = n.addEdge(a, b, 1.0, 60.0);
    const HazardId h1 = n.injectHazard(e, HazardKind::Congestion, 2.0, "");
    const HazardId h2 = n.injectHazard(e, HazardKind::Congestion, 3.0, "");
    CHECK_NEAR(n.edge(e).congestion, 3.0, 1e-9);  // worst wins
    n.resolveHazard(h2);
    CHECK_NEAR(n.edge(e).congestion, 2.0, 1e-9);
    const HazardId h3 = n.injectHazard(e, HazardKind::Closure, 1.0, "");
    CHECK(n.edge(e).blocked);
    n.resolveHazard(h3);
    CHECK(!n.edge(e).blocked);
    n.resolveHazard(h1);
    CHECK_NEAR(n.edge(e).congestion, 1.0, 1e-9);
    CHECK(n.activeHazardCount() == 0);
}

// ------------------------------------------------------------------ routing

RoadNetwork diamond(EdgeId& fastA, EdgeId& fastB, EdgeId& slowB) {
    // 0 -> 1 -> 3 is fast (2 + 2 min); 0 -> 2 -> 3 is slow (6 + 6 min)
    RoadNetwork n;
    for (int i = 0; i < 4; ++i) n.addNode("n" + std::to_string(i), i, 0, NodeKind::Junction);
    fastA = n.addEdge(0, 1, 2.0, 60.0);
    fastB = n.addEdge(1, 3, 2.0, 60.0);
    n.addEdge(0, 2, 6.0, 60.0);
    slowB = n.addEdge(2, 3, 6.0, 60.0);
    return n;
}

void testShortestPathAndRerouting() {
    EdgeId fa, fb, sb;
    RoadNetwork n = diamond(fa, fb, sb);
    Route r = shortestPath(n, 0, 3);
    CHECK(r.reachable);
    CHECK((r.path == std::vector<NodeId>{0, 1, 3}));
    CHECK_NEAR(r.minutes, 4.0, 1e-9);

    n.injectHazard(fb, HazardKind::Closure, 1.0, "");  // V2X closure -> detour
    r = shortestPath(n, 0, 3);
    CHECK(r.reachable);
    CHECK((r.path == std::vector<NodeId>{0, 2, 3}));
    CHECK_NEAR(r.minutes, 12.0, 1e-9);

    n.injectHazard(sb, HazardKind::Closure, 1.0, "");  // both ways closed
    r = shortestPath(n, 0, 3);
    CHECK(!r.reachable);
    CHECK(r.minutes == kInf);

    Route self = shortestPath(n, 2, 2);
    CHECK(self.reachable && self.path.size() == 1 && self.minutes == 0.0);
}

void testStagingNodeSelection() {
    CityData c = buildDemoCity();
    c.net.injectHazard(*c.net.edgeBetween(Harbor, Hillcrest), HazardKind::Closure, 1.0, "");
    CHECK(!shortestPath(c.net, StationBravo, Hillcrest).reachable);
    const StagingPlan sp = findStagingNode(c.net, StationBravo, Hillcrest);
    CHECK(sp.found);
    CHECK(sp.node == Harbor);  // closest reachable node to Hillcrest
    CHECK(sp.route.reachable && sp.route.path.front() == StationBravo && sp.route.path.back() == Harbor);
}

// ------------------------------------------------------------------ dispatch

void testSuitabilityRanking() {
    Simulation sim(buildDemoCity());
    const Dispatcher& d = sim.dispatcher();
    auto severe = d.previewRanking(IncidentType::Medical, 5, OldTown, VehicleType::Ambulance);
    auto minor = d.previewRanking(IncidentType::Medical, 1, OldTown, VehicleType::Ambulance);
    CHECK(severe.size() == 4 && minor.size() == 4);
    // severe call: the advanced-trauma unit (A4, triage 5) wins despite a longer trip
    CHECK(sim.world().vehicle(severe.front().vehicle).callsign == "A4");
    // minor call: proximity dominates, the closest unit (A1) wins
    CHECK(sim.world().vehicle(minor.front().vehicle).callsign == "A1");
    for (std::size_t i = 1; i < severe.size(); ++i) CHECK(severe[i - 1].score >= severe[i].score);

    // fire: bigger tank wins a severe fire, ladder wins a rescue
    auto fire = d.previewRanking(IncidentType::Fire, 5, Central, VehicleType::FireEngine);
    CHECK(sim.world().vehicle(fire.front().vehicle).callsign == "E2");
    auto rescue = d.previewRanking(IncidentType::Rescue, 3, Central, VehicleType::FireEngine);
    CHECK(sim.world().vehicle(rescue.front().vehicle).callsign == "E2");
}

void testRequiredRoles() {
    CHECK(Dispatcher::requiredRoles(IncidentType::Medical, 5).size() == 1);
    CHECK(Dispatcher::requiredRoles(IncidentType::Fire, 3).size() == 1);
    const auto pkg = Dispatcher::requiredRoles(IncidentType::Fire, 4);
    CHECK(pkg.size() == 2);
    CHECK(pkg[0] == VehicleType::FireEngine && pkg[1] == VehicleType::Ambulance);
    CHECK(Dispatcher::requiredRoles(IncidentType::Rescue, 5).size() == 2);
}

void testCoDispatch() {
    Simulation sim(buildDemoCity());
    const IncidentId id = sim.createIncident(IncidentType::Fire, 5, Industrial, "test");
    const Incident& inc = sim.world().incident(id);
    CHECK(inc.assigned.size() == 2);
    int engines = 0, ambulances = 0;
    for (VehicleId v : inc.assigned) {
        if (sim.world().vehicle(v).type == VehicleType::FireEngine) ++engines;
        else ++ambulances;
    }
    CHECK(engines == 1 && ambulances == 1);
    CHECK(sim.world().stats.coDispatches == 1);
    CHECK(inc.status == IncidentStatus::Dispatched);
}

void testCoDispatchIsAtomic() {
    // one ambulance + one engine in the whole fleet
    Simulation sim(cityWith([](const Vehicle& v) { return v.callsign == "A1" || v.callsign == "E1"; }));
    sim.createIncident(IncidentType::Medical, 3, OldTown, "keeps the ambulance busy");
    const IncidentId id = sim.createIncident(IncidentType::Rescue, 4, Central, "needs engine + ambulance");
    const Incident& inc = sim.world().incident(id);
    CHECK(inc.status == IncidentStatus::Queued);
    CHECK(inc.assigned.empty());  // engine must not be taken alone
    CHECK(sim.world().fleet[1].state == VehicleState::IdleStation);
    CHECK(sim.world().queue.size() == 1);
    CHECK(sim.runUntilQuiet(600));  // once the ambulance returns the package is dispatched
    CHECK(sim.world().incident(id).status == IncidentStatus::Resolved);
}

void testPreemption() {
    Simulation sim(oneAmbulanceCity());
    const IncidentId low = sim.createIncident(IncidentType::Medical, 1, OldTown, "minor");
    CHECK(sim.world().fleet[0].incident == low);
    const IncidentId high = sim.createIncident(IncidentType::Medical, 5, Central, "critical");
    CHECK(sim.world().fleet[0].incident == high);
    CHECK(sim.world().incident(low).status == IncidentStatus::Queued);
    CHECK(sim.world().incident(low).preemptedCount == 1);
    CHECK(sim.world().stats.preemptions == 1);
    CHECK(std::find(sim.world().queue.begin(), sim.world().queue.end(), low) != sim.world().queue.end());
    CHECK(sim.runUntilQuiet(600));
    CHECK(sim.world().incident(low).status == IncidentStatus::Resolved);
    CHECK(sim.world().incident(high).status == IncidentStatus::Resolved);
    CHECK(sim.world().incident(high).resolvedAt < sim.world().incident(low).resolvedAt);
}

void testNoPreemptionOfMidSeverityOrBySeverity4() {
    Simulation sim(oneAmbulanceCity());
    const IncidentId mid = sim.createIncident(IncidentType::Medical, 3, OldTown, "mid");
    const IncidentId hi = sim.createIncident(IncidentType::Medical, 5, Central, "critical");
    CHECK(sim.world().stats.preemptions == 0);  // sev 3 is protected
    CHECK(sim.world().incident(hi).status == IncidentStatus::Queued);
    CHECK(sim.world().fleet[0].incident == mid);

    Simulation sim2(oneAmbulanceCity());
    const IncidentId low = sim2.createIncident(IncidentType::Medical, 1, OldTown, "minor");
    const IncidentId four = sim2.createIncident(IncidentType::Medical, 4, Central, "serious");
    CHECK(sim2.world().stats.preemptions == 0);  // only severity 5 may preempt
    CHECK(sim2.world().fleet[0].incident == low);
    CHECK(sim2.world().incident(four).status == IncidentStatus::Queued);
}

void testQueueAgingAndPriorityOrder() {
    Simulation sim(oneAmbulanceCity());
    sim.world().now = 30.0;
    Incident old;
    old.severity = 2;
    old.createdAt = 0.0;
    Incident fresh;
    fresh.severity = 4;
    fresh.createdAt = 30.0;
    CHECK(sim.dispatcher().effectivePriority(old) > sim.dispatcher().effectivePriority(fresh));
    CHECK_NEAR(sim.dispatcher().effectivePriority(old), 5.0, 1e-9);

    Simulation q(oneAmbulanceCity());
    q.createIncident(IncidentType::Medical, 2, OldTown, "occupies the only unit");
    const IncidentId lowId = q.createIncident(IncidentType::Medical, 1, Harbor, "waits");
    q.advance(1.0);
    const IncidentId highId = q.createIncident(IncidentType::Medical, 3, Riverside, "waits too");
    CHECK(q.world().queue.size() == 2);
    CHECK(q.runUntilQuiet(900));
    const Incident& lo = q.world().incident(lowId);
    const Incident& hi = q.world().incident(highId);
    CHECK(hi.firstDispatchAt >= 0.0 && lo.firstDispatchAt >= 0.0);
    CHECK(hi.firstDispatchAt <= lo.firstDispatchAt);  // higher effective priority goes first
    CHECK(q.world().stats.queueWaitSamples == 2);
    CHECK(q.world().stats.meanQueueWait() > 0.0);
}

// ------------------------------------------------------------------ simulation

void testVehicleLifecycle() {
    Simulation sim(oneAmbulanceCity());  // A1 is based at Station Alpha, so it must drive back
    const IncidentId id = sim.createIncident(IncidentType::Medical, 3, NorthHub, "lifecycle");
    const VehicleId vid = sim.world().incident(id).assigned.at(0);
    std::vector<VehicleState> seen;
    for (int step = 0; step < 800; ++step) {
        const VehicleState s = sim.world().vehicle(vid).state;
        if (seen.empty() || seen.back() != s) seen.push_back(s);
        sim.advance(0.25);
        if (sim.allQuiet()) break;
    }
    const VehicleState last = sim.world().vehicle(vid).state;
    if (seen.back() != last) seen.push_back(last);
    const std::vector<VehicleState> expected = {
        VehicleState::EnRouteIncident, VehicleState::OnScene, VehicleState::TransportingHospital,
        VehicleState::AtHospitalTurnover, VehicleState::ReturningToBase, VehicleState::IdleStation};
    CHECK(seen == expected);
    CHECK(sim.world().vehicle(vid).node == sim.world().vehicle(vid).base);
    CHECK(sim.world().vehicle(vid).distanceKm > 0.0);
    CHECK(sim.world().incident(id).status == IncidentStatus::Resolved);
    CHECK(sim.world().stats.incidentsResolved == 1);
}

void testDeterminism() {
    auto run = []() {
        Simulation sim(buildDemoCity());
        sim.createIncident(IncidentType::Fire, 5, Industrial, "a");
        sim.createIncident(IncidentType::Medical, 2, Harbor, "b");
        sim.advance(5.0);
        sim.createIncident(IncidentType::Medical, 5, Central, "c");
        sim.runUntilQuiet(600);
        return sim.world().eventLog;
    };
    CHECK(run() == run());
}

void testDynamicReroutingOnClosure() {
    Simulation sim(buildDemoCity());
    const IncidentId id = sim.createIncident(IncidentType::Medical, 3, Riverside, "reroute");
    const VehicleId vid = sim.world().incident(id).assigned.at(0);
    sim.advance(0.5);
    sim.injectHazard(*sim.world().net.edgeBetween(NorthHub, Riverside), HazardKind::Closure, 1.0, "");
    CHECK(sim.runUntilQuiet(600));
    CHECK(sim.world().stats.reroutes >= 1);
    CHECK(sim.world().incident(id).status == IncidentStatus::Resolved);
    CHECK(sim.world().vehicle(vid).state == VehicleState::IdleStation);
}

void testCongestionSlowsInFlightVehicle() {
    auto responseWith = [](double multiplier) {
        Simulation sim(oneAmbulanceCity());
        const IncidentId id = sim.createIncident(IncidentType::Medical, 3, NorthHub, "x");
        // A1 leaves Station Alpha towards North Hub; congest that very road
        sim.injectHazard(*sim.world().net.edgeBetween(StationAlpha, NorthHub), HazardKind::Congestion,
                         multiplier, "");
        sim.runUntilQuiet(600);
        const Incident& inc = sim.world().incident(id);
        return inc.firstArrivalAt - inc.createdAt;
    };
    CHECK_NEAR(responseWith(1.0), 2.0, 1e-6);   // 2 km @ 60 km/h
    CHECK_NEAR(responseWith(3.0), 6.0, 1e-6);   // remaining distance is stretched x3
}

void testSignalPreemptionSpeedsSevereCalls() {
    auto response = [](int severity, VehicleId& unit) {
        Simulation sim(buildDemoCity());
        const IncidentId id = sim.createIncident(IncidentType::Medical, severity, NorthHub, "x");
        unit = sim.world().incident(id).assigned.at(0);
        sim.runUntilQuiet(600);
        const Incident& inc = sim.world().incident(id);
        return inc.firstArrivalAt - inc.createdAt;
    };
    VehicleId u3 = -1, u4 = -1;
    const double r3 = response(3, u3);
    const double r4 = response(4, u4);
    CHECK(u3 == u4);
    CHECK(r4 < r3);
    CHECK_NEAR(r4 / r3, DispatchPolicy().signalPreemptionFactor, 1e-6);
}

void testPerimeterStagingAndResume() {
    Simulation sim(buildDemoCity());
    const HazardId h = sim.injectHazard(*sim.world().net.edgeBetween(Harbor, Hillcrest),
                                        HazardKind::Closure, 1.0, "landslide");
    const IncidentId id = sim.createIncident(IncidentType::Fire, 3, Hillcrest, "cut off");
    const VehicleId vid = sim.world().incident(id).assigned.at(0);
    sim.advance(30.0);
    CHECK(sim.world().vehicle(vid).state == VehicleState::StagedAtPerimeter);
    CHECK(sim.world().vehicle(vid).node == Harbor);
    CHECK(sim.world().stats.stagings == 1);
    CHECK(sim.world().incident(id).status == IncidentStatus::Dispatched);

    sim.resolveHazard(h);
    CHECK(sim.world().stats.resumes == 1);
    CHECK(sim.world().vehicle(vid).state == VehicleState::EnRouteIncident);
    CHECK(sim.runUntilQuiet(600));
    CHECK(sim.world().incident(id).status == IncidentStatus::Resolved);
    CHECK(sim.world().incident(id).firstArrivalAt > 30.0);
}

void testClosureWhileEnRouteToIsolatedNode() {
    // The road closes while the engine is already travelling: it must divert to staging.
    Simulation sim(buildDemoCity());
    const IncidentId id = sim.createIncident(IncidentType::Fire, 3, Hillcrest, "x");
    const VehicleId vid = sim.world().incident(id).assigned.at(0);
    sim.advance(1.0);
    const HazardId h = sim.injectHazard(*sim.world().net.edgeBetween(Harbor, Hillcrest),
                                        HazardKind::Closure, 1.0, "");
    sim.advance(40.0);
    CHECK(sim.world().vehicle(vid).state == VehicleState::StagedAtPerimeter);
    sim.resolveHazard(h);
    CHECK(sim.runUntilQuiet(600));
}

void testScenarioRunnerAndTelemetry() {
    Simulation sim(buildDemoCity());
    sim.createIncident(IncidentType::Fire, 4, Industrial, "json \"quote\" test");
    sim.injectHazard(*sim.world().net.edgeBetween(Central, EastHub), HazardKind::Congestion, 2.0, "x");
    sim.advance(10.0);
    const std::string js = toJson(sim.world());
    for (const char* key : {"\"time_minutes\"", "\"kpis\"", "\"nodes\"", "\"edges\"", "\"hazards\"",
                            "\"vehicles\"", "\"incidents\"", "\"queue\""})
        CHECK(js.find(key) != std::string::npos);
    CHECK(js.find("json \\\"quote\\\" test") == std::string::npos);  // description is not exported
    int depth = 0;
    bool inString = false, escape = false, ok = true;
    for (char ch : js) {
        if (inString) {
            if (escape) escape = false;
            else if (ch == '\\') escape = true;
            else if (ch == '"') inString = false;
            continue;
        }
        if (ch == '"') inString = true;
        else if (ch == '{' || ch == '[') ++depth;
        else if (ch == '}' || ch == ']') { if (--depth < 0) ok = false; }
    }
    CHECK(ok && depth == 0 && !inString);

    std::ostringstream text;
    printDashboard(text, sim.world());
    CHECK(text.str().find("COMMAND CENTER") != std::string::npos);

    for (int i = 1; i <= kScenarioCount; ++i) {  // every scripted scenario ends quiet
        std::ostringstream sink;
        runScenario(i, sink);
        CHECK(sink.str().find("incidents created/resolved") != std::string::npos);
    }
}

void testResetAndInvalidInput() {
    Simulation sim(buildDemoCity());
    sim.createIncident(IncidentType::Medical, 3, OldTown, "x");
    sim.advance(5.0);
    sim.reset();
    CHECK(sim.now() == 0.0);
    CHECK(sim.world().incidents.empty());
    CHECK(sim.world().stats.incidentsCreated == 0);
    CHECK(sim.world().fleet.size() == 7);
    CHECK(sim.world().fleet[0].state == VehicleState::IdleStation);

    bool threw = false;
    try { sim.createIncident(IncidentType::Medical, 9, OldTown); } catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { sim.createIncident(IncidentType::Medical, 3, 999); } catch (const std::out_of_range&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { sim.advance(-1.0); } catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
}

void testAStarRouting() {
    const CityData c = buildDemoCity();
    const std::vector<std::pair<NodeId, NodeId>> pairs = {
        {Central, Riverside}, {StationAlpha, Hillcrest}, {OldTown, Harbor},
        {Central, Central}, {WestHub, EastsideMedical}
    };
    for (const auto& p : pairs) {
        const Route dRoute = shortestPath(c.net, p.first, p.second);
        const Route aRoute = aStar(c.net, p.first, p.second);
        CHECK(dRoute.reachable == aRoute.reachable);
        if (dRoute.reachable) {
            CHECK_NEAR(dRoute.minutes, aRoute.minutes, 1e-6);
            CHECK(dRoute.path.size() == aRoute.path.size());
            CHECK(aRoute.path.front() == p.first);
            CHECK(aRoute.path.back() == p.second);
        }
    }
}

void testOneWayRoads() {
    RoadNetwork net;
    const NodeId a = net.addNode("A", 0, 0, NodeKind::Junction);
    const NodeId b = net.addNode("B", 1, 0, NodeKind::Junction);
    net.addEdge(a, b, 1.0, 60.0, true);  // One-way: A -> B

    const Route ab = shortestPath(net, a, b);
    CHECK(ab.reachable);
    CHECK(ab.path.size() == 2);

    const Route ba = shortestPath(net, b, a);
    CHECK(!ba.reachable);  // Traversal against one-way direction is disallowed
}

void testHospitalDivert() {
    CityData c = buildDemoCity();
    Hospital cg;
    cg.node = CityGeneral;
    cg.onDivert = true;  // City General is on divert
    Hospital em;
    em.node = EastsideMedical;
    em.onDivert = false;
    c.hospitals = {cg, em};

    Simulation sim(c);
    // When City General is on divert (+30 min penalty), Eastside Medical becomes the best choice
    const NodeId chosen = sim.chooseHospital(Central);
    CHECK(chosen == EastsideMedical);
}

void testWaterDepletionAndRefill() {
    CityData c = buildDemoCity();
    c.fleet.erase(std::remove_if(c.fleet.begin(), c.fleet.end(),
                                 [](const Vehicle& v) { return v.callsign != "E1"; }),
                  c.fleet.end());
    Simulation sim(c);
    CHECK(sim.world().fleet[0].waterLiters == 3000.0);
    CHECK(sim.world().fleet[0].maxWaterLiters == 3000.0);

    // Create fire incident: severity 3 -> uses 3 * 400 = 1200 L water
    sim.createIncident(IncidentType::Fire, 3, NorthHub, "fire scene");
    sim.advance(5.0);  // E1 arrives on scene
    CHECK(sim.world().fleet[0].state == VehicleState::OnScene);
    CHECK_NEAR(sim.world().fleet[0].waterLiters, 1800.0, 1e-6);

    // After resolving scene and returning to station, water tank refills
    CHECK(sim.runUntilQuiet(120));
    CHECK(sim.world().fleet[0].state == VehicleState::IdleStation);
    CHECK_NEAR(sim.world().fleet[0].waterLiters, 3000.0, 1e-6);
}

struct TestCase {
    const char* name;
    void (*fn)();
};

}  // namespace

int main() {
    const std::vector<TestCase> tests = {
        {"network construction", testNetworkConstruction},
        {"travel time formula", testTravelTimeFormula},
        {"overlapping hazards", testOverlappingHazards},
        {"shortest path + dynamic rerouting", testShortestPathAndRerouting},
        {"staging node selection", testStagingNodeSelection},
        {"suitability ranking", testSuitabilityRanking},
        {"required roles", testRequiredRoles},
        {"heterogeneous co-dispatch", testCoDispatch},
        {"co-dispatch is atomic", testCoDispatchIsAtomic},
        {"severity-5 preemption", testPreemption},
        {"preemption limits", testNoPreemptionOfMidSeverityOrBySeverity4},
        {"queue aging + priority order", testQueueAgingAndPriorityOrder},
        {"vehicle lifecycle", testVehicleLifecycle},
        {"determinism", testDeterminism},
        {"dynamic reroute on closure", testDynamicReroutingOnClosure},
        {"congestion slows in-flight unit", testCongestionSlowsInFlightVehicle},
        {"signal preemption", testSignalPreemptionSpeedsSevereCalls},
        {"perimeter staging + resume", testPerimeterStagingAndResume},
        {"closure while en route", testClosureWhileEnRouteToIsolatedNode},
        {"telemetry + scenarios", testScenarioRunnerAndTelemetry},
        {"reset + invalid input", testResetAndInvalidInput},
        {"A* routing algorithm", testAStarRouting},
        {"one-way roads", testOneWayRoads},
        {"hospital divert & routing", testHospitalDivert},
        {"water depletion and refill", testWaterDepletionAndRefill},
    };
    int failedTests = 0;
    for (const TestCase& t : tests) {
        const int before = g_failed;
        try {
            t.fn();
        } catch (const std::exception& e) {
            ++g_failed;
            std::cerr << "  EXCEPTION in '" << t.name << "': " << e.what() << "\n";
        }
        const bool ok = g_failed == before;
        if (!ok) ++failedTests;
        std::cout << (ok ? "[ PASS ] " : "[ FAIL ] ") << t.name << "\n";
    }
    std::cout << "\n" << (tests.size() - static_cast<std::size_t>(failedTests)) << "/" << tests.size()
              << " tests passed, " << g_checks << " checks, " << g_failed << " failed\n";
    return g_failed == 0 ? 0 : 1;
}

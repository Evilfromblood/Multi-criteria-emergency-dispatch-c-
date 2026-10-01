#include "scenario.hpp"

#include <iomanip>
#include <stdexcept>

#include "telemetry.hpp"

namespace v2x {

namespace {

Vehicle makeAmbulance(const char* callsign, NodeId base, int triage, bool paramedic) {
    Vehicle v;
    v.callsign = callsign;
    v.type = VehicleType::Ambulance;
    v.base = base;
    v.node = base;
    v.triageLevel = triage;
    v.paramedic = paramedic;
    return v;
}

Vehicle makeEngine(const char* callsign, NodeId base, double waterLiters, bool ladder) {
    Vehicle v;
    v.callsign = callsign;
    v.type = VehicleType::FireEngine;
    v.base = base;
    v.node = base;
    v.waterLiters = waterLiters;
    v.ladder = ladder;
    return v;
}

void banner(std::ostream& out, int number) {
    out << "\n#############################################################\n"
        << "# SCENARIO " << number << ": " << scenarioTitle(number) << "\n"
        << "#############################################################\n";
}

void printRanking(std::ostream& out, const Simulation& sim, const char* title,
                  const std::vector<Candidate>& cands) {
    const World& w = sim.world();
    out << title << "\n";
    out << "  " << std::left << std::setw(6) << "Unit" << std::setw(10) << "ETA(min)" << std::setw(9)
        << "Score" << "Profile\n";
    for (const Candidate& c : cands) {
        const Vehicle& v = w.vehicle(c.vehicle);
        out << "  " << std::left << std::setw(6) << v.callsign << std::fixed << std::setprecision(1)
            << std::setw(10) << c.eta << std::setprecision(2) << std::setw(9) << c.score;
        if (v.type == VehicleType::Ambulance)
            out << "triage " << v.triageLevel << (v.paramedic ? ", paramedic" : ", no paramedic");
        else
            out << static_cast<int>(v.waterLiters) << " L water" << (v.ladder ? ", ladder" : "");
        out << "\n";
    }
}

EdgeId road(const Simulation& sim, NodeId a, NodeId b) {
    const auto e = sim.world().net.edgeBetween(a, b);
    if (!e) throw std::logic_error("scenario references a non-existent road");
    return *e;
}

}  // namespace

CityData buildDemoCity() {
    CityData c;
    RoadNetwork& n = c.net;
    using namespace city;
    // Order must match the enum in scenario.hpp.
    n.addNode("Central", 5, 5, NodeKind::Junction);
    n.addNode("North Hub", 5, 8, NodeKind::Junction);
    n.addNode("East Hub", 8, 5, NodeKind::Junction);
    n.addNode("South Hub", 5, 2, NodeKind::Junction);
    n.addNode("West Hub", 2, 5, NodeKind::Junction);
    n.addNode("Riverside", 8, 8, NodeKind::District);
    n.addNode("Old Town", 2, 8, NodeKind::District);
    n.addNode("Harbor", 8, 2, NodeKind::District);
    n.addNode("Industrial Park", 2, 2, NodeKind::District);
    n.addNode("Station Alpha", 5, 10, NodeKind::Station);
    n.addNode("Station Bravo", 10, 5, NodeKind::Station);
    n.addNode("Station Charlie", 5, 0, NodeKind::Station);
    n.addNode("City General", 6, 6, NodeKind::Hospital);
    n.addNode("Eastside Medical", 10, 8, NodeKind::Hospital);
    n.addNode("Hillcrest", 11, 2, NodeKind::District);

    n.addEdge(Central, NorthHub, 3.0, 50);
    n.addEdge(Central, EastHub, 3.0, 50);
    n.addEdge(Central, SouthHub, 3.0, 50);
    n.addEdge(Central, WestHub, 3.0, 50);
    n.addEdge(Central, CityGeneral, 1.5, 40);
    n.addEdge(NorthHub, OldTown, 3.2, 40);
    n.addEdge(NorthHub, Riverside, 3.2, 40);
    n.addEdge(NorthHub, StationAlpha, 2.0, 60);
    n.addEdge(NorthHub, CityGeneral, 2.4, 40);
    n.addEdge(EastHub, Riverside, 3.2, 40);
    n.addEdge(EastHub, Harbor, 3.4, 40);
    n.addEdge(EastHub, StationBravo, 2.0, 60);
    n.addEdge(EastHub, CityGeneral, 2.5, 40);
    n.addEdge(SouthHub, Harbor, 3.2, 40);
    n.addEdge(SouthHub, Industrial, 3.2, 40);
    n.addEdge(SouthHub, StationCharlie, 2.0, 60);
    n.addEdge(WestHub, OldTown, 3.2, 40);
    n.addEdge(WestHub, Industrial, 3.2, 40);
    n.addEdge(Riverside, EastsideMedical, 2.5, 40);
    n.addEdge(StationBravo, EastsideMedical, 3.1, 50);
    n.addEdge(Harbor, Hillcrest, 3.1, 40);  // the only road into Hillcrest

    c.fleet.push_back(makeAmbulance("A1", StationAlpha, 3, true));
    c.fleet.push_back(makeAmbulance("A2", StationBravo, 2, false));
    c.fleet.push_back(makeAmbulance("A3", StationCharlie, 4, true));
    c.fleet.push_back(makeAmbulance("A4", CityGeneral, 5, true));
    c.fleet.push_back(makeEngine("E1", StationAlpha, 3000, false));
    c.fleet.push_back(makeEngine("E2", StationBravo, 4500, true));
    c.fleet.push_back(makeEngine("E3", StationCharlie, 2500, false));
    return c;
}

const char* scenarioTitle(int number) {
    switch (number) {
        case 1: return "Multi-criteria suitability ranking";
        case 2: return "V2X dynamic rerouting (closure + congestion)";
        case 3: return "Heterogeneous co-dispatch and atomicity";
        case 4: return "Severity-5 preemption, queue aging and starvation guard";
        case 5: return "Perimeter staging and automatic resumption";
        default: return "unknown";
    }
}

void runScenario(int number, std::ostream& out) {
    if (number < 1 || number > kScenarioCount) throw std::out_of_range("no such scenario");
    using namespace city;
    banner(out, number);
    Simulation sim(buildDemoCity());
    sim.setEcho(&out);
    Dispatcher& d = sim.dispatcher();

    switch (number) {
        case 1: {
            out << "Same location (Old Town), different severity: the winner changes.\n\n";
            printRanking(out, sim, "Ranking for MEDICAL severity 1:",
                         d.previewRanking(IncidentType::Medical, 1, OldTown, VehicleType::Ambulance));
            printRanking(out, sim, "\nRanking for MEDICAL severity 5:",
                         d.previewRanking(IncidentType::Medical, 5, OldTown, VehicleType::Ambulance));
            out << "\nEvents:\n";
            sim.createIncident(IncidentType::Medical, 5, OldTown, "cardiac arrest");
            sim.createIncident(IncidentType::Medical, 1, OldTown, "minor injury");
            sim.advance(3.0);
            printDashboard(out, sim.world());
            sim.runUntilQuiet(240);
            break;
        }
        case 2: {
            sim.createIncident(IncidentType::Medical, 3, Riverside, "road traffic injury");
            sim.schedule(1.0, "closure", [&](Simulation& s) {
                s.injectHazard(road(s, NorthHub, Riverside), HazardKind::Closure, 1.0,
                               "overturned truck");
            });
            sim.schedule(4.0, "congestion", [&](Simulation& s) {
                s.injectHazard(road(s, Central, EastHub), HazardKind::Congestion, 3.0,
                               "stadium traffic");
            });
            sim.advance(2.5);
            printDashboard(out, sim.world());
            sim.runUntilQuiet(240);
            break;
        }
        case 3: {
            sim.createIncident(IncidentType::Fire, 5, Industrial, "chemical warehouse fire");
            sim.createIncident(IncidentType::Medical, 3, Harbor, "dockside fall");
            sim.createIncident(IncidentType::Medical, 3, OldTown, "chest pain");
            sim.createIncident(IncidentType::Medical, 3, Riverside, "allergic reaction");
            out << "\nAll four ambulances are now committed. A severity-4 rescue needs\n"
                   "1 engine + 1 ambulance; idle engines must NOT be taken until both exist.\n\n";
            sim.createIncident(IncidentType::Rescue, 4, OldTown, "building collapse");
            printDashboard(out, sim.world());
            sim.runUntilQuiet(480);
            break;
        }
        case 4: {
            sim.createIncident(IncidentType::Medical, 2, Harbor, "sprain");
            sim.createIncident(IncidentType::Medical, 2, Industrial, "fall");
            sim.createIncident(IncidentType::Medical, 2, WestHub, "fever");
            sim.createIncident(IncidentType::Medical, 2, Riverside, "minor burn");
            sim.createIncident(IncidentType::Medical, 1, OldTown, "check-up request");
            sim.advance(2.0);
            out << "\nFleet saturated, severity-1 call waiting. A severity-5 call arrives:\n";
            sim.createIncident(IncidentType::Medical, 5, Central, "multi-casualty collision");
            printDashboard(out, sim.world());
            sim.runUntilQuiet(480);
            break;
        }
        case 5: {
            sim.injectHazard(road(sim, Harbor, Hillcrest), HazardKind::Closure, 1.0, "landslide");
            sim.createIncident(IncidentType::Fire, 3, Hillcrest, "forest fire near homes");
            sim.advance(20.0);
            printDashboard(out, sim.world());
            const HazardId landslide = 0;
            sim.schedule(sim.now() + 5.0, "clear", [landslide](Simulation& s) { s.resolveHazard(landslide); });
            sim.runUntilQuiet(480);
            break;
        }
        default: break;
    }

    out << "\n--- FINAL STATE ---";
    printDashboard(out, sim.world());
}

}  // namespace v2x

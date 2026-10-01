#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>

#include "scenario.hpp"
#include "telemetry.hpp"

using namespace v2x;

namespace {

std::optional<std::string> readLine(const std::string& prompt) {
    std::cout << prompt << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) return std::nullopt;
    return line;
}

// Returns nullopt on end of input. Re-asks until the value is in [lo, hi].
std::optional<int> askInt(const std::string& prompt, int lo, int hi) {
    for (;;) {
        const auto line = readLine(prompt);
        if (!line) return std::nullopt;
        try {
            std::size_t used = 0;
            const int value = std::stoi(*line, &used);
            if (used == line->size() && value >= lo && value <= hi) return value;
        } catch (const std::exception&) {
        }
        std::cout << "  please enter a whole number between " << lo << " and " << hi << "\n";
    }
}

std::optional<double> askDouble(const std::string& prompt, double lo, double hi) {
    for (;;) {
        const auto line = readLine(prompt);
        if (!line) return std::nullopt;
        try {
            std::size_t used = 0;
            const double value = std::stod(*line, &used);
            if (used == line->size() && value >= lo && value <= hi) return value;
        } catch (const std::exception&) {
        }
        std::cout << "  please enter a number between " << lo << " and " << hi << "\n";
    }
}

void printMenu() {
    std::cout << "\n---------------- V2X EMERGENCY DISPATCH ----------------\n"
                 " 1  Dashboard                    6  Run until all quiet\n"
                 " 2  New incident                 7  Run a scripted scenario\n"
                 " 3  Inject V2X hazard            8  Export telemetry JSON\n"
                 " 4  Resolve V2X hazard           9  Show road network / event log\n"
                 " 5  Advance simulation time     10  Reset simulation\n"
                 " 0  Exit\n"
                 "--------------------------------------------------------\n";
}

void printUsage() {
    std::cout << "Usage: emergency_dispatch [--demo | --scenario N | --help]\n"
                 "  (no arguments)  interactive console menu\n"
                 "  --demo          run all scripted scenarios\n"
                 "  --scenario N    run scenario N (1.." << kScenarioCount << ")\n";
}

bool newIncident(Simulation& sim) {
    std::cout << "Type: 1 Medical  2 Fire  3 Rescue\n";
    const auto t = askInt("type> ", 1, 3);
    if (!t) return false;
    const auto sev = askInt("severity (1-5)> ", 1, 5);
    if (!sev) return false;
    printNetwork(std::cout, sim.world());
    const auto node = askInt("location node id> ", 0, static_cast<int>(sim.world().net.nodeCount()) - 1);
    if (!node) return false;
    const IncidentType type = *t == 1 ? IncidentType::Medical : *t == 2 ? IncidentType::Fire : IncidentType::Rescue;
    sim.createIncident(type, *sev, *node, "operator entry");
    return true;
}

bool injectHazard(Simulation& sim) {
    printNetwork(std::cout, sim.world());
    const auto edge = askInt("road id> ", 0, static_cast<int>(sim.world().net.edgeCount()) - 1);
    if (!edge) return false;
    const auto kind = askInt("1 Congestion  2 Closure> ", 1, 2);
    if (!kind) return false;
    double mult = 1.0;
    if (*kind == 1) {
        const auto m = askDouble("congestion multiplier (1.0-10.0)> ", 1.0, 10.0);
        if (!m) return false;
        mult = *m;
    }
    sim.injectHazard(*edge, *kind == 1 ? HazardKind::Congestion : HazardKind::Closure, mult, "operator entry");
    return true;
}

bool resolveHazard(Simulation& sim) {
    printHazards(std::cout, sim.world());
    const auto id = askInt("hazard id> ", 0, 1000000);
    if (!id) return false;
    if (!sim.resolveHazard(*id)) std::cout << "  no active hazard with that id\n";
    return true;
}

bool scenarioMenu() {
    for (int i = 1; i <= kScenarioCount; ++i) std::cout << " " << i << "  " << scenarioTitle(i) << "\n";
    std::cout << " 0  back\n";
    const auto n = askInt("scenario> ", 0, kScenarioCount);
    if (!n) return false;
    if (*n > 0) runScenario(*n, std::cout);
    return true;
}

void interactive() {
    Simulation sim(buildDemoCity());
    sim.setEcho(&std::cout);
    for (;;) {
        printMenu();
        const auto choice = askInt("choice> ", 0, 10);
        if (!choice || *choice == 0) return;
        bool ok = true;
        try {
            switch (*choice) {
                case 1: printDashboard(std::cout, sim.world()); break;
                case 2: ok = newIncident(sim); break;
                case 3: ok = injectHazard(sim); break;
                case 4: ok = resolveHazard(sim); break;
                case 5: {
                    const auto m = askDouble("minutes to advance (0-600)> ", 0.0, 600.0);
                    if (!m) return;
                    sim.advance(*m);
                    break;
                }
                case 6:
                    std::cout << (sim.runUntilQuiet(1440) ? "System quiet.\n" : "Not quiet after 24 h of simulated time.\n");
                    break;
                case 7: ok = scenarioMenu(); break;
                case 8: {
                    std::ofstream f("telemetry.json");
                    f << toJson(sim.world());
                    std::cout << (f ? "Wrote telemetry.json\n" : "Could not write telemetry.json\n");
                    break;
                }
                case 9: {
                    printNetwork(std::cout, sim.world());
                    const auto& log = sim.world().eventLog;
                    const std::size_t from = log.size() > 25 ? log.size() - 25 : 0;
                    std::cout << "\nLAST EVENTS\n";
                    for (std::size_t i = from; i < log.size(); ++i) std::cout << "  " << log[i] << "\n";
                    break;
                }
                case 10:
                    sim.reset();
                    std::cout << "Simulation reset.\n";
                    break;
                default: break;
            }
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << "\n";
        }
        if (!ok) return;  // end of input
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc >= 2) {
            const std::string arg = argv[1];
            if (arg == "--demo") {
                for (int i = 1; i <= kScenarioCount; ++i) runScenario(i, std::cout);
                return 0;
            }
            if (arg == "--scenario" && argc >= 3) {
                runScenario(std::atoi(argv[2]), std::cout);
                return 0;
            }
            printUsage();
            return arg == "--help" ? 0 : 1;
        }
        interactive();
    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 2;
    }
    return 0;
}

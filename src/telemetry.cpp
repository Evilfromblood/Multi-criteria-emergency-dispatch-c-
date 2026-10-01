#include "telemetry.hpp"

#include <iomanip>
#include <sstream>

#include "motion.hpp"

namespace v2x {

namespace {

std::string jsonEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        switch (c) {
            case '"': o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;
            case '\t': o += "\\t"; break;
            default: o += c;
        }
    }
    return o;
}

std::string q(const std::string& s) { return "\"" + jsonEscape(s) + "\""; }

std::string num(double v) {
    if (v == kInf) return "null";
    std::ostringstream o;
    o << std::fixed << std::setprecision(2) << v;
    return o.str();
}

std::string location(const World& w, const Vehicle& v) {
    if (v.midHop) return w.nodeName(v.node) + " > " + w.nodeName(v.nextNode);
    return w.nodeName(v.node);
}

std::string etaText(const World& w, const Vehicle& v) {
    double m = -1.0;
    if (v.midHop) m = remainingTravelMinutes(w, v);
    else if (hasTimer(v.state)) m = v.timer;
    if (m < 0.0) return "-";
    std::ostringstream o;
    o << std::fixed << std::setprecision(1) << m;
    return o.str();
}

std::string unitList(const World& w, const Incident& inc) {
    std::string s;
    for (VehicleId id : inc.assigned) s += (s.empty() ? "" : ",") + w.vehicle(id).callsign;
    return s.empty() ? "-" : s;
}

}  // namespace

std::string toJson(const World& w) {
    std::ostringstream o;
    o << "{\n";
    o << "  \"time_minutes\": " << num(w.now) << ",\n";

    const Analytics& a = w.stats;
    o << "  \"kpis\": {"
      << "\"incidents_created\": " << a.incidentsCreated
      << ", \"incidents_resolved\": " << a.incidentsResolved
      << ", \"dispatches\": " << a.dispatches
      << ", \"co_dispatches\": " << a.coDispatches
      << ", \"preemptions\": " << a.preemptions
      << ", \"stagings\": " << a.stagings
      << ", \"resumes\": " << a.resumes
      << ", \"reroutes\": " << a.reroutes
      << ", \"queue_depth\": " << w.queue.size()
      << ", \"max_queue_depth\": " << a.maxQueueDepth
      << ", \"mean_response_minutes\": " << num(a.meanResponse())
      << ", \"mean_queue_wait_minutes\": " << num(a.meanQueueWait()) << "},\n";

    o << "  \"nodes\": [";
    for (std::size_t i = 0; i < w.net.nodes().size(); ++i) {
        const Node& n = w.net.nodes()[i];
        o << (i ? ", " : "") << "{\"id\": " << n.id << ", \"name\": " << q(n.name)
          << ", \"kind\": " << q(toString(n.kind)) << ", \"x\": " << num(n.x)
          << ", \"y\": " << num(n.y) << "}";
    }
    o << "],\n  \"edges\": [";
    for (std::size_t i = 0; i < w.net.edges().size(); ++i) {
        const Edge& e = w.net.edges()[i];
        o << (i ? ", " : "") << "{\"id\": " << e.id << ", \"a\": " << e.a << ", \"b\": " << e.b
          << ", \"distance_km\": " << num(e.distanceKm) << ", \"speed_kmh\": " << num(e.speedKmh)
          << ", \"congestion\": " << num(e.congestion)
          << ", \"blocked\": " << (e.blocked ? "true" : "false")
          << ", \"travel_minutes\": " << num(e.travelMinutes()) << "}";
    }
    o << "],\n  \"hazards\": [";
    bool first = true;
    for (const Hazard& h : w.net.hazards()) {
        if (!h.active) continue;
        o << (first ? "" : ", ") << "{\"id\": " << h.id << ", \"edge\": " << h.edge
          << ", \"kind\": " << q(toString(h.kind)) << ", \"multiplier\": " << num(h.multiplier)
          << ", \"description\": " << q(h.description) << "}";
        first = false;
    }
    o << "],\n  \"vehicles\": [";
    for (std::size_t i = 0; i < w.fleet.size(); ++i) {
        const Vehicle& v = w.fleet[i];
        o << (i ? ",\n    " : "\n    ") << "{\"id\": " << v.id << ", \"callsign\": " << q(v.callsign)
          << ", \"type\": " << q(toString(v.type)) << ", \"state\": " << q(toString(v.state))
          << ", \"node\": " << v.node << ", \"next_node\": " << v.nextNode
          << ", \"destination\": " << v.destination << ", \"incident\": " << v.incident
          << ", \"distance_km\": " << num(v.distanceKm) << ", \"busy_minutes\": " << num(v.busyMinutes)
          << "}";
    }
    o << "\n  ],\n  \"incidents\": [";
    for (std::size_t i = 0; i < w.incidents.size(); ++i) {
        const Incident& inc = w.incidents[i];
        o << (i ? ",\n    " : "\n    ") << "{\"id\": " << inc.id << ", \"type\": " << q(toString(inc.type))
          << ", \"severity\": " << inc.severity << ", \"location\": " << inc.location
          << ", \"status\": " << q(toString(inc.status)) << ", \"created_at\": " << num(inc.createdAt)
          << ", \"first_arrival_at\": " << num(inc.firstArrivalAt)
          << ", \"resolved_at\": " << num(inc.resolvedAt)
          << ", \"preempted\": " << inc.preemptedCount << ", \"assigned\": [";
        for (std::size_t k = 0; k < inc.assigned.size(); ++k)
            o << (k ? ", " : "") << inc.assigned[k];
        o << "]}";
    }
    o << "\n  ],\n  \"queue\": [";
    for (std::size_t i = 0; i < w.queue.size(); ++i) o << (i ? ", " : "") << w.queue[i];
    o << "]\n}\n";
    return o.str();
}

void printFleet(std::ostream& out, const World& w) {
    out << "FLEET\n";
    out << std::left << std::setw(6) << "Unit" << std::setw(13) << "Type" << std::setw(23) << "State"
        << std::setw(28) << "Location" << std::setw(18) << "Destination" << std::setw(6) << "Inc"
        << "ETA/Timer(min)\n";
    out << std::string(106, '-') << "\n";
    for (const Vehicle& v : w.fleet) {
        out << std::left << std::setw(6) << v.callsign << std::setw(13) << toString(v.type)
            << std::setw(23) << toString(v.state) << std::setw(28) << location(w, v)
            << std::setw(18) << w.nodeName(v.destination)
            << std::setw(6) << (v.incident < 0 ? std::string("-") : "#" + std::to_string(v.incident))
            << etaText(w, v) << "\n";
    }
}

void printIncidents(std::ostream& out, const World& w) {
    out << "INCIDENTS\n";
    if (w.incidents.empty()) {
        out << "  (none)\n";
        return;
    }
    out << std::left << std::setw(6) << "ID" << std::setw(9) << "Type" << std::setw(5) << "Sev"
        << std::setw(18) << "Location" << std::setw(12) << "Status" << std::setw(14) << "Units"
        << std::setw(9) << "Created" << std::setw(11) << "Response" << "Notes\n";
    out << std::string(94, '-') << "\n";
    for (const Incident& inc : w.incidents) {
        std::ostringstream resp;
        if (inc.firstArrivalAt >= 0.0)
            resp << std::fixed << std::setprecision(1) << (inc.firstArrivalAt - inc.createdAt);
        else
            resp << "-";
        std::string notes;
        if (inc.status == IncidentStatus::Queued) notes += "waiting ";
        if (inc.preemptedCount > 0) notes += "preempted x" + std::to_string(inc.preemptedCount);
        out << std::left << std::setw(6) << ("#" + std::to_string(inc.id)) << std::setw(9)
            << toString(inc.type) << std::setw(5) << inc.severity << std::setw(18)
            << w.nodeName(inc.location) << std::setw(12) << toString(inc.status) << std::setw(14)
            << unitList(w, inc) << std::setw(9) << std::fixed << std::setprecision(1) << inc.createdAt
            << std::setw(11) << resp.str() << notes << "\n";
    }
}

void printHazards(std::ostream& out, const World& w) {
    out << "V2X HAZARDS\n";
    bool any = false;
    for (const Hazard& h : w.net.hazards()) {
        if (!h.active) continue;
        const Edge& e = w.net.edge(h.edge);
        out << "  #" << h.id << " " << toString(h.kind) << " on " << w.nodeName(e.a) << " <-> "
            << w.nodeName(e.b);
        if (h.kind == HazardKind::Congestion) out << " (x" << std::fixed << std::setprecision(1) << h.multiplier << ")";
        if (!h.description.empty()) out << " - " << h.description;
        out << "\n";
        any = true;
    }
    if (!any) out << "  (none active)\n";
}

void printKpis(std::ostream& out, const World& w) {
    const Analytics& a = w.stats;
    double util = 0.0;
    if (!w.fleet.empty() && w.now > 0.0) {
        for (const Vehicle& v : w.fleet) util += v.busyMinutes / w.now;
        util = 100.0 * util / static_cast<double>(w.fleet.size());
    }
    out << "KPIs  t=" << std::fixed << std::setprecision(1) << w.now << " min\n";
    out << "  incidents created/resolved : " << a.incidentsCreated << " / " << a.incidentsResolved << "\n";
    out << "  queue depth (max)          : " << w.queue.size() << " (" << a.maxQueueDepth << ")\n";
    out << "  unit dispatches            : " << a.dispatches << "  (co-dispatch packages: " << a.coDispatches << ")\n";
    out << "  preemptions                : " << a.preemptions << "\n";
    out << "  stagings / resumes         : " << a.stagings << " / " << a.resumes << "\n";
    out << "  dynamic reroutes           : " << a.reroutes << "\n";
    out << "  mean response time         : " << a.meanResponse() << " min\n";
    out << "  mean queue wait            : " << a.meanQueueWait() << " min\n";
    out << "  fleet utilisation          : " << util << " %\n";
}

void printDashboard(std::ostream& out, const World& w) {
    out << "\n==================== COMMAND CENTER ====================\n";
    printKpis(out, w);
    out << "\n";
    printFleet(out, w);
    out << "\n";
    printIncidents(out, w);
    out << "\n";
    printHazards(out, w);
    out << "========================================================\n";
}

void printNetwork(std::ostream& out, const World& w) {
    out << "NODES\n";
    for (const Node& n : w.net.nodes())
        out << "  " << std::setw(2) << n.id << "  " << std::left << std::setw(20) << n.name << std::right
            << toString(n.kind) << "\n";
    out << "ROADS\n";
    for (const Edge& e : w.net.edges()) {
        out << "  " << std::setw(2) << e.id << "  " << std::left << std::setw(18) << w.nodeName(e.a)
            << " <-> " << std::setw(18) << w.nodeName(e.b) << std::right << std::fixed
            << std::setprecision(1) << e.distanceKm << " km @ " << e.speedKmh << " km/h";
        if (e.blocked) out << "  [CLOSED]";
        else if (e.congestion > 1.0) out << "  [congestion x" << e.congestion << "]";
        out << "\n";
    }
}

}  // namespace v2x

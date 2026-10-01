#pragma once
#include <limits>

namespace v2x {

using NodeId = int;
using EdgeId = int;
using VehicleId = int;
using IncidentId = int;
using HazardId = int;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-9;

enum class NodeKind { Junction, District, Station, Hospital };
enum class VehicleType { Ambulance, FireEngine };
enum class VehicleState {
    IdleStation,
    EnRouteIncident,
    StagedAtPerimeter,
    OnScene,
    TransportingHospital,
    AtHospitalTurnover,
    ReturningToBase
};
enum class IncidentType { Medical, Fire, Rescue };
enum class IncidentStatus { Queued, Dispatched, OnScene, Resolved };
enum class HazardKind { Congestion, Closure };

inline const char* toString(NodeKind k) {
    switch (k) {
        case NodeKind::Junction: return "JUNCTION";
        case NodeKind::District: return "DISTRICT";
        case NodeKind::Station: return "STATION";
        case NodeKind::Hospital: return "HOSPITAL";
    }
    return "?";
}
inline const char* toString(VehicleType t) {
    return t == VehicleType::Ambulance ? "AMBULANCE" : "FIRE_ENGINE";
}
inline const char* toString(VehicleState s) {
    switch (s) {
        case VehicleState::IdleStation: return "IDLE_STATION";
        case VehicleState::EnRouteIncident: return "EN_ROUTE_INCIDENT";
        case VehicleState::StagedAtPerimeter: return "STAGED_AT_PERIMETER";
        case VehicleState::OnScene: return "ON_SCENE";
        case VehicleState::TransportingHospital: return "TRANSPORTING_HOSPITAL";
        case VehicleState::AtHospitalTurnover: return "AT_HOSPITAL_TURNOVER";
        case VehicleState::ReturningToBase: return "RETURNING_TO_BASE";
    }
    return "?";
}
inline const char* toString(IncidentType t) {
    switch (t) {
        case IncidentType::Medical: return "MEDICAL";
        case IncidentType::Fire: return "FIRE";
        case IncidentType::Rescue: return "RESCUE";
    }
    return "?";
}
inline const char* toString(IncidentStatus s) {
    switch (s) {
        case IncidentStatus::Queued: return "QUEUED";
        case IncidentStatus::Dispatched: return "DISPATCHED";
        case IncidentStatus::OnScene: return "ON_SCENE";
        case IncidentStatus::Resolved: return "RESOLVED";
    }
    return "?";
}
inline const char* toString(HazardKind k) {
    return k == HazardKind::Closure ? "CLOSURE" : "CONGESTION";
}

}  // namespace v2x

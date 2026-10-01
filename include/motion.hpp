#pragma once
#include "router.hpp"
#include "world.hpp"

namespace v2x {

inline bool isMovingState(VehicleState s) {
    return s == VehicleState::EnRouteIncident || s == VehicleState::TransportingHospital ||
           s == VehicleState::ReturningToBase;
}
inline bool hasTimer(VehicleState s) {
    return s == VehicleState::OnScene || s == VehicleState::AtHospitalTurnover;
}

// The node from which a *new* route must start (the next node if mid-hop),
// and how many minutes until the vehicle gets there.
inline NodeId anchorNode(const Vehicle& v) { return v.midHop ? v.nextNode : v.node; }
inline double anchorDelay(const Vehicle& v) { return v.midHop ? v.hopRemaining : 0.0; }

// Install a route (route.path[0] must equal anchorNode(v)). Starts the first
// hop immediately when the vehicle is standing at a node.
void setRoute(World& w, Vehicle& v, const Route& route);

// Minutes still needed to reach the end of the current plan under today's
// road conditions, or -1 when the vehicle is not travelling / plan is blocked.
double remainingTravelMinutes(const World& w, const Vehicle& v);

}  // namespace v2x

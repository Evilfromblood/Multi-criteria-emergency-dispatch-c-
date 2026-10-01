#include "world.hpp"

#include <cstdio>

namespace v2x {

std::string World::nodeName(NodeId id) const {
    return id < 0 ? std::string("-") : net.node(id).name;
}

void World::log(const std::string& message) {
    char stamp[32];
    std::snprintf(stamp, sizeof stamp, "[t=%7.2f] ", now);
    const std::string line = std::string(stamp) + message;
    eventLog.push_back(line);
    if (echo) (*echo) << line << '\n';
}

}  // namespace v2x

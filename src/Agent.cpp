#include "UtiasMrclam/agents/Agent.hpp"

namespace utias::mrclam {

Agent::Agent(unsigned short id, unsigned short barcode, Type type)
    : id_{id}, barcode_{barcode}, type_{type} {}

const Agent::ID Agent::id() const { return id_; }
const Agent::Barcode Agent::barcode() const { return barcode_; }
const Agent::Type Agent::type() const { return type_; }

} // namespace utias::mrclam

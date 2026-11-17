#include "GlobalContext.hpp"

namespace apa {

void GlobalStates::reset() noexcept {}

namespace global {
GlobalParams params{};
GlobalStates states{};
}

}

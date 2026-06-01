#include "ethercat/backend.hpp"

namespace ethercat {

const char* to_string(EcatState state) noexcept {
    switch (state) {
        case EcatState::None:
            return "None";
        case EcatState::Init:
            return "Init";
        case EcatState::PreOp:
            return "PreOp";
        case EcatState::SafeOp:
            return "SafeOp";
        case EcatState::Op:
            return "Op";
    }
    return "Unknown";
}

}  // namespace ethercat

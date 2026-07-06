#include "viam/lib/motion_profile.hpp"

#include <algorithm>

namespace ethercat::servo {

double clamp_rpm(double rpm, double max_rpm) noexcept {
    const double bound = max_rpm > 0.0 ? max_rpm : 0.0;
    return std::clamp(rpm, -bound, bound);
}

}  // namespace ethercat::servo

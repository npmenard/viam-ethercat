#include <cstdint>

#include "test_harness.hpp"
#include "viam/lib/motion_profile.hpp"

namespace mp = ethercat::servo;

namespace {
constexpr double kA6CountsPerRev = 131072.0;  // 2^17 absolute encoder
}  // namespace

TEST("revs<->counts round-trip and gear ratio") {
    CHECK_EQ(mp::revs_to_counts(1.0, kA6CountsPerRev, 1.0), std::int32_t{131072});
    CHECK_EQ(mp::revs_to_counts(2.0, kA6CountsPerRev, 1.0), std::int32_t{262144});
    CHECK_EQ(mp::revs_to_counts(0.5, kA6CountsPerRev, 1.0), std::int32_t{65536});
    CHECK_EQ(mp::revs_to_counts(1.0, kA6CountsPerRev, 10.0), std::int32_t{1310720});  // 10:1 gear
    CHECK_EQ(mp::revs_to_counts(-0.25, kA6CountsPerRev, 1.0), std::int32_t{-32768});  // negative
    CHECK_EQ(mp::counts_to_revs(131072, kA6CountsPerRev, 1.0), 1.0);
    CHECK_EQ(mp::counts_to_revs(1310720, kA6CountsPerRev, 10.0), 1.0);
}

TEST("revs_to_counts saturates instead of overflowing int32") {
    CHECK_EQ(mp::revs_to_counts(1e9, kA6CountsPerRev, 1.0), std::int32_t{2147483647});
    CHECK_EQ(mp::revs_to_counts(-1e9, kA6CountsPerRev, 1.0), std::int32_t{-2147483648});
}

TEST("clamp_rpm bounds magnitude symmetrically (sign-preserving)") {
    CHECK_EQ(mp::clamp_rpm(3500.0, 3000.0), 3000.0);
    CHECK_EQ(mp::clamp_rpm(-3500.0, 3000.0), -3000.0);
    CHECK_EQ(mp::clamp_rpm(1500.0, 3000.0), 1500.0);
    CHECK_EQ(mp::clamp_rpm(1500.0, -1.0), 0.0);  // negative bound -> fully clamped
}

TEST("rpm<->device velocity (counts/s = rpm/60 * counts_per_rev * gear)") {
    CHECK_EQ(mp::rpm_to_device_velocity(60.0, kA6CountsPerRev, 1.0), std::int32_t{131072});
    CHECK_EQ(mp::rpm_to_device_velocity(30.0, kA6CountsPerRev, 1.0), std::int32_t{65536});
    CHECK_EQ(mp::rpm_to_device_velocity(60.0, kA6CountsPerRev, 2.0), std::int32_t{262144});  // gear
    CHECK_EQ(mp::device_velocity_to_rpm(131072, kA6CountsPerRev, 1.0), 60.0);
    CHECK_EQ(mp::device_velocity_to_rpm(262144, kA6CountsPerRev, 2.0), 60.0);
}

TEST("amps -> torque per-mille (A6: rated-current scaling, clamp 0..4000)") {
    CHECK_EQ(mp::amps_to_torque_permille(2.5, 2.5), std::uint16_t{1000});   // rated -> 1000
    CHECK_EQ(mp::amps_to_torque_permille(1.25, 2.5), std::uint16_t{500});   // half rated
    CHECK_EQ(mp::amps_to_torque_permille(5.0, 2.5), std::uint16_t{2000});   // 2x rated
    CHECK_EQ(mp::amps_to_torque_permille(20.0, 2.5), std::uint16_t{4000});  // clamps at 4000
    CHECK_EQ(mp::amps_to_torque_permille(0.0, 2.5), std::uint16_t{0});
    CHECK_EQ(mp::amps_to_torque_permille(5.0, 0.0), std::uint16_t{0});  // no rated current -> 0
}

TEST_MAIN()

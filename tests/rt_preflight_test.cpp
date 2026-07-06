// The realtime requirement, tested the way it will actually be hit: the module is usually
// loaded on a host with NO realtime scheduling (stock desktop/server, CI included), and the
// user must get a clear, actionable error at load -- before any bus I/O -- not a misleading
// drive error later. These tests are environment-branching by design: CI (no SCHED_FIFO
// permission) exercises the refusal path; a realtime-privileged box (root / rtprio rlimit)
// exercises the pass-through path. Both branches are asserted, so the test is green
// everywhere while still pinning the behavior that matters for the environment it runs in.

#include <pthread.h>
#include <sched.h>

#include <string>

#include "ethercat/errors.hpp"
#include "ethercat/realtime.hpp"
#include "test_harness.hpp"
#include "viam/lib/servo_config.hpp"
#include "viam/lib/servo_controller.hpp"

using ethercat::Error;
using ethercat::servo::rt_unavailable_message;
using ethercat::servo::ServoConfig;
using ethercat::servo::ServoController;

namespace {

ServoConfig valid_config() {
    ServoConfig c;
    // An interface name that can never open, so a preflight that wrongly passes fails later
    // with a DIFFERENT (NIC) error text -- the assertions below tell the two apart.
    c.ifname = "no-such-nic0";
    c.slave_id = 1;
    c.max_motor_speed_rpm = 3000.0;
    c.motor_rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = 131072.0;
    c.position_tolerance_counts = 10;
    c.target_loop_rate_hz = 1000;
    c.rt_priority = 80;
    c.command_queue_capacity = 64;
    return c;
}

}  // namespace

TEST("rt_unavailable_message names the failure, the fix, and the opt-out") {
    const std::string msg = rt_unavailable_message(80);
    // What failed, at which priority.
    CHECK(msg.find("SCHED_FIFO") != std::string::npos);
    CHECK(msg.find("80") != std::string::npos);
    // How to fix it (viam-server realtime permission; PREEMPT_RT recommendation).
    CHECK(msg.find("viam-server") != std::string::npos);
    CHECK(msg.find("PREEMPT_RT") != std::string::npos);
    // The explicit development opt-out, spelled exactly like the config attribute.
    CHECK(msg.find("\"require_realtime\": false") != std::string::npos);
}

TEST("sched_fifo_available: no side effects on the calling thread, stable answer") {
    int policy_before = 0;
    sched_param param_before{};
    CHECK(pthread_getschedparam(pthread_self(), &policy_before, &param_before) == 0);

    const bool first = ethercat::realtime::sched_fifo_available(80);
    const bool second = ethercat::realtime::sched_fifo_available(80);
    CHECK_EQ(first, second);  // a probe, not a coin flip

    // The probe runs on a scratch thread; this thread's scheduling must be untouched.
    int policy_after = 0;
    sched_param param_after{};
    CHECK(pthread_getschedparam(pthread_self(), &policy_after, &param_after) == 0);
    CHECK_EQ(policy_after, policy_before);
    CHECK_EQ(param_after.sched_priority, param_before.sched_priority);
}

TEST("require_realtime on a non-RT host: start() throws the clear realtime error BEFORE bus I/O") {
    const bool rt_ok = ethercat::realtime::sched_fifo_available(80);
    ServoConfig cfg = valid_config();
    cfg.require_realtime = true;
    ServoController c(std::move(cfg));
    if (!rt_ok) {
        // The CI / stock-host branch: the preflight must throw the user-facing realtime error.
        // Matching the opt-out phrase proves it is rt_unavailable_message, not a NIC error --
        // i.e. the throw happened before open("no-such-nic0") was ever attempted.
        CHECK_THROWS_MSG(c.start(), Error, "require_realtime");
    } else {
        // Realtime-privileged branch (root / rtprio rlimit): the preflight passes and start()
        // proceeds to the bus, failing on the bogus NIC -- NOT with the realtime error.
        try {
            c.start();
            CHECK(false);  // "no-such-nic0" must not open
        } catch (const Error& e) {
            CHECK(std::string(e.what()).find("require_realtime") == std::string::npos);
        }
    }
}

TEST("require_realtime=false: the realtime gate is skipped (start() reaches the bus)") {
    ServoConfig cfg = valid_config();
    cfg.require_realtime = false;
    ServoController c(std::move(cfg));
    // With the gate off, start() must get past the preflight on ANY host and fail at the bogus
    // NIC instead. Asserting the message is not the realtime text pins the opt-out.
    try {
        c.start();
        CHECK(false);  // "no-such-nic0" must not open
    } catch (const Error& e) {
        CHECK(std::string(e.what()).find("require_realtime") == std::string::npos);
    }
}

TEST_MAIN()

#include "ethercat/sim_backend.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"

namespace ethercat {

namespace {

// CiA402 device-side statusword base for each state (matches Status::decode()).
std::uint16_t statusword_base(Cia402State state) noexcept {
    switch (state) {
        case Cia402State::NotReadyToSwitchOn:
            return 0x0000;
        case Cia402State::SwitchOnDisabled:
            return 0x0040;
        case Cia402State::ReadyToSwitchOn:
            return 0x0021;
        case Cia402State::SwitchedOn:
            return 0x0023;
        case Cia402State::OperationEnabled:
            return 0x0027;
        case Cia402State::QuickStopActive:
            return 0x0007;
        case Cia402State::FaultReactionActive:
            return 0x000F;
        case Cia402State::Fault:
            return 0x0008;
    }
    return 0x0000;
}

std::uint32_t sdo_key(std::uint16_t index, std::uint8_t sub) noexcept {
    return (static_cast<std::uint32_t>(index) << 8U) | sub;
}

}  // namespace

SimBackend::SimBackend(std::vector<SimSlaveModel> slaves) {
    // emplace in place: Slave holds atomics (cross-thread test-hook fields) so it is
    // non-movable; std::deque back-insertion constructs directly without moving.
    for (auto& model : slaves) {
        slaves_.emplace_back();
        slaves_.back().model = std::move(model);
    }
}

std::size_t SimBackend::open(std::string_view ifname) {
    if (open_) {
        throw ConfigError("SimBackend::open: bus already open on '" + std::string(ifname) + "' (one master per backend; close() first)");
    }
    if (slaves_.empty()) {
        throw InitError("SimBackend::open: no simulated slaves configured on '" + std::string(ifname) + "'");
    }
    open_ = true;
    for (auto& slave : slaves_) {
        slave.state = EcatState::PreOp;
        slave.device_state = Cia402State::NotReadyToSwitchOn;
    }
    return slaves_.size();
}

SlaveInfo SimBackend::slave_info(std::uint16_t slave) const {
    if (slave < 1 || slave > slaves_.size()) {
        throw ConfigError("SimBackend::slave_info: slave " + std::to_string(slave) + " out of range (1.." + std::to_string(slaves_.size()) +
                          ")");
    }
    const Slave& s = slaves_[slave - 1];
    SlaveInfo info;
    info.position = slave;
    info.vendor_id = s.model.vendor_id;
    info.product_code = s.model.product_code;
    info.name = s.model.name;
    info.input_bytes = s.model.input_bytes;
    info.output_bytes = s.model.output_bytes;
    return info;
}

void SimBackend::sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) {
    if (slave < 1 || slave > slaves_.size()) {
        throw ConfigError("SimBackend::sdo_write: slave " + std::to_string(slave) + " out of range (configured " +
                          std::to_string(slaves_.size()) + ")");
    }
    Slave& s = slaves_[slave - 1];
    // Test injection (#32 note 14): simulate a drive CoE abort on this object. SdoError is the
    // GENERIC SDO tier; apply_pdo_map re-tags it PdoMappingError for the mapping objects.
    if (const auto it = s.sdo_write_aborts.find(sdo_key(index, sub)); it != s.sdo_write_aborts.end()) {
        throw SdoError("SimBackend: slave " + std::to_string(slave) + " aborted SDO write to object " + std::to_string(index) + ":" +
                       std::to_string(sub) + " (CoE abort code " + std::to_string(it->second) + ")");
    }
    s.dictionary[sdo_key(index, sub)] = std::vector<std::byte>(data.begin(), data.end());
    s.sdo_write_order.push_back(sdo_key(index, sub));
    // De-mask: the runtime mode of operation comes from the 0x6060 SDO (U8), NOT
    // model.mode -- so a master that forgets to set it leaves the device in mode 0.
    if (index == 0x6060 && sub == 0 && !data.empty()) {
        s.effective_mode = static_cast<Cia402Mode>(static_cast<std::int8_t>(data[0]));
    }
}

std::size_t SimBackend::sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
    if (slave < 1 || slave > slaves_.size()) {
        throw ConfigError("SimBackend::sdo_read: slave " + std::to_string(slave) + " out of range (configured " +
                          std::to_string(slaves_.size()) + ")");
    }
    const Slave& s = slaves_[slave - 1];
    // #53: 0x605A (quick-stop option, i16) is a device object the control READS to assert the
    // PV stop regime -- serve it from the model (default 2). 0x6085 (quick-stop decel) normally
    // reads back the written value (dictionary, below), but the model can FORCE a clamp/absent
    // echo (0 -> the control refuses; a clamped value -> the control uses the echoed value).
    if (index == 0x605A && sub == 0 && out.size() >= 2) {
        store_le<std::int16_t>(out.subspan(0, 2), s.model.quick_stop_option);
        return 2;
    }
    if (index == 0x6085 && sub == 0 && s.model.quick_stop_decel_echo_forced && out.size() >= 4) {
        store_le<std::uint32_t>(out.subspan(0, 4), s.model.quick_stop_decel_echo);
        return 4;
    }
    const auto it = s.dictionary.find(sdo_key(index, sub));
    if (it == s.dictionary.end()) {
        return 0;
    }
    const std::size_t n = std::min(out.size(), it->second.size());
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = it->second[i];
    }
    return n;
}

void SimBackend::map_process_data() {
    expected_wkc_ = 0;
    for (auto& s : slaves_) {
        // Validate model offsets fit the image sizes at SETUP time. step_device()
        // is noexcept and uses subspan(), so a bad offset there would throw and
        // std::terminate -- fail loudly here instead.
        const SimSlaveModel& m = s.model;
        const bool fault_oob = m.fault_code_off >= 0 && static_cast<std::size_t>(m.fault_code_off) + 2 > m.input_bytes;
        const bool vel_oob = m.velocity_actual_off >= 0 && static_cast<std::size_t>(m.velocity_actual_off) + 4 > m.input_bytes;
        if (m.ctrlword_off + 2 > m.output_bytes || (m.target_off + 4 > m.output_bytes && m.mode == Cia402Mode::ProfilePosition) ||
            m.statusword_off + 2 > m.input_bytes || m.actual_off + 4 > m.input_bytes || fault_oob || vel_oob) {
            throw ConfigError("SimSlaveModel offsets exceed the image sizes (out=" + std::to_string(m.output_bytes) +
                              ", in=" + std::to_string(m.input_bytes) + ")");
        }
        s.output_image.assign(s.model.output_bytes, std::byte{0});
        s.input_image.assign(s.model.input_bytes, std::byte{0});
        // Each slave with both a command and feedback image contributes 3 to the
        // working counter (2 write + 1 read), mirroring SOEM's accounting.
        expected_wkc_ += (s.model.output_bytes > 0 ? 2 : 0) + (s.model.input_bytes > 0 ? 1 : 0);
        s.state = EcatState::SafeOp;
    }
}

void SimBackend::request_state(std::uint16_t slave, EcatState target) {
    if (target == EcatState::Op) {
        ++op_requests_;  // #47 test hook: the no-hammer metric (exactly one per start)
    }
    if (slave == 0) {
        for (auto& s : slaves_) {
            s.state = target;
        }
        return;
    }
    if (slave > slaves_.size()) {
        throw ConfigError("SimBackend::request_state: slave " + std::to_string(slave) + " out of range (configured " +
                          std::to_string(slaves_.size()) + ")");
    }
    slaves_[slave - 1].state = target;
}

EcatState SimBackend::slave_state(std::uint16_t slave) const {
    if (slave == 0) {
        // Relies on EcatState being ordered least->most progressed
        // (None<Init<PreOp<SafeOp<Op)); do not reorder the enum.
        EcatState worst = EcatState::Op;
        for (const auto& s : slaves_) {
            worst = std::min(worst, s.state);
        }
        return slaves_.empty() ? EcatState::None : worst;
    }
    if (slave > slaves_.size()) {
        return EcatState::None;
    }
    return slaves_[slave - 1].state;
}

SlaveIo SimBackend::slave_io(std::uint16_t slave) noexcept {
    if (slave < 1 || slave > slaves_.size()) {
        return {};
    }
    Slave& s = slaves_[slave - 1];
    return SlaveIo{std::span<std::byte>(s.output_image), std::span<const std::byte>(s.input_image)};
}

void SimBackend::step_device(Slave& s) noexcept {
    const auto out = std::span<const std::byte>(s.output_image);
    const std::uint16_t cw = load_le<std::uint16_t>(out.subspan(s.model.ctrlword_off, 2));
    const std::uint16_t prev = s.prev_ctrlword;

    // CiA402 command decode (controlword masks).
    const bool shutdown = (cw & 0x87U) == 0x06U;
    const bool switch_on = (cw & 0x8FU) == 0x07U;
    const bool disable_voltage = (cw & 0x82U) == 0x00U;
    const bool enable_op = (cw & 0x8FU) == 0x0FU;
    const bool quick_stop = (cw & 0x86U) == 0x02U;
    const bool fault_reset_rising = ((cw & 0x80U) != 0U) && ((prev & 0x80U) == 0U);
    if (fault_reset_rising) {
        s.fault_reset_edges.fetch_add(1, std::memory_order_relaxed);  // #18 no-spin observability
    }

    using St = Cia402State;
    // #18 type-(c): a momentary clear re-faults after its hold elapses (drive accepted
    // the reset, resumed, re-detected the cause).
    if (s.refault_countdown_ > 0) {
        if (--s.refault_countdown_ == 0) {
            s.faulted.store(true, std::memory_order_relaxed);
        }
    }
    if (s.faulted.load(std::memory_order_relaxed) && s.device_state != St::Fault) {
        s.device_state = St::Fault;
        // #18: a fresh fault starts clean reflect/refault countdowns (RT-side, race-free).
        s.clear_countdown_ = 0;
        s.refault_countdown_ = 0;
    }

    switch (s.device_state) {
        case St::NotReadyToSwitchOn:
            s.device_state = St::SwitchOnDisabled;  // auto power-on advance
            break;
        case St::SwitchOnDisabled:
            if (shutdown) {
                s.device_state = St::ReadyToSwitchOn;
            }
            break;
        case St::ReadyToSwitchOn:
            if (switch_on) {
                s.device_state = St::SwitchedOn;
            } else if (disable_voltage) {
                s.device_state = St::SwitchOnDisabled;
            }
            break;
        case St::SwitchedOn:
            if (enable_op) {
                s.device_state = St::OperationEnabled;
            } else if (shutdown) {
                s.device_state = St::ReadyToSwitchOn;
            } else if (disable_voltage) {
                s.device_state = St::SwitchOnDisabled;
            }
            break;
        case St::OperationEnabled:
            if (quick_stop) {
                s.device_state = St::QuickStopActive;
            } else if (enable_op) {
                // stay in OperationEnabled
            } else if (switch_on) {
                s.device_state = St::SwitchedOn;  // disable operation
            } else if (shutdown) {
                s.device_state = St::ReadyToSwitchOn;
            } else if (disable_voltage) {
                s.device_state = St::SwitchOnDisabled;
            }
            break;
        case St::QuickStopActive:
            if (disable_voltage) {
                // #53 control-driven exit (the 0x605A in {5,6,7} backstop, OR the no-op cw=0x00
                // the control sends after the =2 auto-disable): record the 0x606C velocity at the
                // de-energize so the test can prove it ramped to ~0 FIRST (not a torque-cut).
                s.velocity_at_qsa_exit = s.pv_velocity;
                s.device_state = St::SwitchOnDisabled;
            } else if (enable_op) {
                s.device_state = St::OperationEnabled;
            }
            break;
        case St::FaultReactionActive:
            // The sim doesn't model the transient fault-reaction state
            // (inject_fault jumps straight to Fault); kept for completeness. The
            // master's Cia402Fsm::step handles it either way.
            s.device_state = St::Fault;
            break;
        case St::Fault: {
            // #18: model the drive's Fault->Switch-On-Disabled clear behaviour.
            //  - persistent cause (type-b): the reset edge is ignored, stays Fault.
            //  - reflect latency `d` (type-a): accept the edge, reflect the clear after d
            //    exchanges (d=0 = instant, the unchanged default / common case).
            //  - clear-then-refault (type-c): on clearing, arm a re-fault after `hold`.
            if (s.fault_persistent.load(std::memory_order_relaxed)) {
                break;  // cause still active -- no reset clears it
            }
            const auto do_clear = [&s] {
                s.faulted.store(false, std::memory_order_relaxed);
                s.device_state = St::SwitchOnDisabled;
                const std::uint32_t hold = s.clear_then_refault_hold.load(std::memory_order_relaxed);
                if (hold > 0) {
                    s.refault_countdown_ = hold;  // type-(c): arm the momentary-clear re-fault
                }
            };
            if (fault_reset_rising && s.clear_countdown_ == 0) {
                const std::uint32_t d = s.fault_clear_delay.load(std::memory_order_relaxed);
                if (d == 0) {
                    do_clear();  // instant (unchanged default)
                } else {
                    s.clear_countdown_ = d;  // accept now, reflect the clear after d cycles
                }
            } else if (s.clear_countdown_ > 0) {
                if (--s.clear_countdown_ == 0) {  // type-(a) reflect-delay elapsed
                    do_clear();
                }
            }
            break;
        }
    }

    // Profile-Position set-point-acknowledge handshake (bit4 / bit12). Mode comes
    // from the 0x6060 SDO (effective_mode), NOT model.mode -- mode 0 => no handshake.
    if (s.effective_mode == Cia402Mode::ProfilePosition && s.device_state == St::OperationEnabled) {
        const bool bit4 = (cw & 0x10U) != 0U;
        const bool prev_bit4 = (prev & 0x10U) != 0U;
        if (bit4 && !prev_bit4) {
            s.target = load_le<std::int32_t>(out.subspan(s.model.target_off, 4));
            s.setpoint_ack = !s.suppress_ack.load(std::memory_order_relaxed);  // test hook: withhold bit12 -> handshake times out
        } else if (!bit4 && prev_bit4) {
            s.setpoint_ack = false;
        }
    } else {
        s.setpoint_ack = false;
    }

    // Motion: chase the target (PP) or integrate velocity (PV). Driven by the
    // SDO-set effective_mode -- in mode 0 (0x6060 never written) the motor does NOT
    // move, even when OperationEnabled, so a missing mode set fails offline.
    const std::int32_t actual_before_motion = s.actual;
    if (s.device_state == St::OperationEnabled) {
        // Record the commanded profile velocity (0x6081) the master wrote -- test
        // visibility for "did the RT loop actually send the move speed?".
        if (s.model.profile_velocity_off >= 0) {
            s.profile_velocity = load_le<std::int32_t>(out.subspan(static_cast<std::size_t>(s.model.profile_velocity_off), 4));
        }
        if (s.effective_mode == Cia402Mode::ProfilePosition) {
            // De-mask: chase at the 0x6081 profile-velocity WIRE value the master
            // wrote (counts/cycle here), NOT a config shortcut -- so if the RT loop
            // forgets to write 0x6081 the value is 0 and the move makes NO progress
            // (caught offline). Fall back to counts_per_step only when 0x6081 isn't
            // mapped at all (profile_velocity_off < 0).
            const std::int32_t step = (s.model.profile_velocity_off >= 0) ? s.profile_velocity : s.model.counts_per_step;
            if (s.actual < s.target) {
                const std::int32_t next = static_cast<std::int32_t>(s.actual + step);
                s.actual = (next > s.target) ? s.target : next;
            } else if (s.actual > s.target) {
                const std::int32_t next = static_cast<std::int32_t>(s.actual - step);
                s.actual = (next < s.target) ? s.target : next;
            }
        } else if (s.effective_mode == Cia402Mode::ProfileVelocity && s.model.velocity_off >= 0) {
            // De-mask: record the commanded 0x60FF (test visibility), and (toy) take the
            // velocity instantly (no accel ramp -- only the quick-stop DECEL ramp matters
            // for the #53 safety assertion). actual integrates the velocity per cycle.
            s.target_velocity = load_le<std::int32_t>(out.subspan(static_cast<std::size_t>(s.model.velocity_off), 4));
            s.pv_velocity = s.target_velocity;
            s.actual = static_cast<std::int32_t>(s.actual + s.pv_velocity);
        }
    } else if (s.device_state == St::QuickStopActive) {
        // #53 Quick-Stop DECEL: ramp |pv_velocity| toward 0 by the model's per-cycle step
        // (0 = instant), still INTEGRATING the (shrinking) velocity -> an ENERGIZED decel that
        // emits a decreasing 0x606C, exactly the ramp-then-disable the PV stop must achieve.
        s.entered_qsa = true;
        const std::int32_t step = s.model.quick_stop_decel_step;
        if (step <= 0 || std::abs(s.pv_velocity) <= step) {
            s.pv_velocity = 0;
        } else {
            s.pv_velocity += (s.pv_velocity > 0) ? -step : step;
        }
        s.actual = static_cast<std::int32_t>(s.actual + s.pv_velocity);
        // 0x605A == 2 (PRIMARY): once the drive reaches its own zero it AUTO-transitions
        // QuickStopActive -> SwitchOnDisabled (the control's cw->0x00 is then a no-op).
        // quick_stop_suppress_auto_disable models a drive that does NOT auto-disable -> the
        // control's cw->0x00 BACKSTOP must do it (tested under a passing 0x605A=2).
        if (s.pv_velocity == 0 && s.model.quick_stop_option == 2 && !s.model.quick_stop_suppress_auto_disable) {
            s.velocity_at_qsa_exit = 0;  // driver-owned de-energize, at zero
            s.device_state = St::SwitchOnDisabled;
        }
    } else {
        s.pv_velocity = 0;  // not energized / not moving
    }
    s.velocity = static_cast<std::int32_t>(s.actual - actual_before_motion);  // per-cycle delta -> 0x606C feedback

    // Compose the statusword.
    unsigned sw = statusword_base(s.device_state);
    sw |= 0x0200U;  // bit9 remote
    if (s.model.target_reached_always_set) {
        sw |= 0x0400U;  // bit10 ALWAYS 1 -- the A6 quirk, modeled per-slave (#43); default = conformant (not forced)
    }
    if (s.device_state != St::SwitchOnDisabled && s.device_state != St::NotReadyToSwitchOn) {
        sw |= 0x0010U;  // bit4 voltage enabled
    }
    if (s.setpoint_ack) {
        sw |= 0x1000U;  // bit12 set-point acknowledge
    }

    const auto in = std::span<std::byte>(s.input_image);
    store_le<std::uint16_t>(in.subspan(s.model.statusword_off, 2), static_cast<std::uint16_t>(sw));
    store_le<std::int32_t>(in.subspan(s.model.actual_off, 4), s.actual);
    // #16 TxPDO feedback de-mask (only when the field is mapped): velocity-actual
    // (0x606C) every cycle from the wire-driven motion; drive error code (0x603F) =
    // the configured code WHILE in Fault, else 0 (so the flag gates the payload).
    if (s.model.velocity_actual_off >= 0) {
        store_le<std::int32_t>(in.subspan(static_cast<std::size_t>(s.model.velocity_actual_off), 4), s.velocity);
    }
    if (s.model.fault_code_off >= 0) {
        // A forced stale code (test hook) overrides the gating -> 0x603F is nonzero
        // even with bit3 clear; otherwise the code is the configured value WHILE in
        // Fault, else 0 (the flag gates the payload on the wire).
        const std::uint16_t stale = s.stale_fault_code.load(std::memory_order_relaxed);
        std::uint16_t code = stale;  // forced stale code overrides the gating
        if (stale == 0 && s.device_state == St::Fault) {
            code = s.fault_code.load(std::memory_order_relaxed);  // gated: code only while faulted
        }
        store_le<std::uint16_t>(in.subspan(static_cast<std::size_t>(s.model.fault_code_off), 2), code);
    }
    // #53 mode-display echo (0x6061, i8): normally the SDO-set effective_mode (the A6 reflects
    // the accepted 0x6060); the model can FORCE a wrong value to model the A6 SILENTLY ignoring
    // an unsupported mode-set (#45) -- the controller's DA-B echo gate must refuse to enable.
    if (s.model.mode_display_off >= 0) {
        const std::int8_t md = s.model.mode_echo_forced ? s.model.mode_echo_value : static_cast<std::int8_t>(s.effective_mode);
        in[static_cast<std::size_t>(s.model.mode_display_off)] = static_cast<std::byte>(md);
    }

    s.prev_ctrlword = cw;
}

int SimBackend::exchange() noexcept {
    if (!open_) {
        return -1;
    }
    synthetic_dc_ns_ += (dc_cycle_ns_ != 0 ? static_cast<std::int64_t>(dc_cycle_ns_) : 1'000'000);  // advance the synthetic DC clock
    for (auto& s : slaves_) {
        // Only in OP are the RxPDO outputs live, so only then does the device
        // consume the controlword and advance its CiA402 state.
        if (s.state == EcatState::Op) {
            step_device(s);
        }
    }
    if (short_wkc_once_ || short_wkc_sticky_.load(std::memory_order_relaxed)) {
        short_wkc_once_ = false;
        return expected_wkc_ - 1;
    }
    return expected_wkc_;
}

int SimBackend::expected_wkc() const noexcept {
    return expected_wkc_;
}

void SimBackend::close() noexcept {
    open_ = false;
    for (auto& s : slaves_) {
        s.state = EcatState::Init;
    }
}

void SimBackend::inject_fault(std::uint16_t slave) noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        slaves_[slave - 1].faulted.store(true, std::memory_order_relaxed);
    }
}

void SimBackend::set_fault_code(std::uint16_t slave, std::uint16_t code) noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        slaves_[slave - 1].fault_code.store(code, std::memory_order_relaxed);
    }
}

void SimBackend::set_stale_fault_code(std::uint16_t slave, std::uint16_t code) noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        slaves_[slave - 1].stale_fault_code.store(code, std::memory_order_relaxed);
    }
}

void SimBackend::set_fault_clear_delay(std::uint16_t slave, std::uint32_t cycles) noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        slaves_[slave - 1].fault_clear_delay.store(cycles, std::memory_order_relaxed);
    }
}

void SimBackend::set_fault_persistent(std::uint16_t slave, bool on) noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        slaves_[slave - 1].fault_persistent.store(on, std::memory_order_relaxed);
    }
}

void SimBackend::set_fault_clear_then_refault(std::uint16_t slave, std::uint32_t hold_cycles) noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        slaves_[slave - 1].clear_then_refault_hold.store(hold_cycles, std::memory_order_relaxed);
    }
}

std::uint32_t SimBackend::fault_reset_edge_count(std::uint16_t slave) const noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        return slaves_[slave - 1].fault_reset_edges.load(std::memory_order_relaxed);
    }
    return 0;
}

void SimBackend::force_short_wkc_once() noexcept {
    short_wkc_once_ = true;
}

void SimBackend::force_short_wkc(bool on) noexcept {
    short_wkc_sticky_.store(on, std::memory_order_relaxed);
}

void SimBackend::suppress_setpoint_ack(std::uint16_t slave, bool on) noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        slaves_[slave - 1].suppress_ack.store(on, std::memory_order_relaxed);
    }
}

void SimBackend::arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) {
    dc_cycle_ns_ = cycle_ns;              // record that the bring-up armed SYNC0 (configured_dc_cycle_ns)
    dc_sync0_shift_ns_ = sync0_shift_ns;  // record the CyclShift the config threaded through (#32 note 4)
}

std::uint32_t SimBackend::configured_dc_cycle_ns() const noexcept {
    return dc_cycle_ns_;
}

std::int32_t SimBackend::configured_dc_sync0_shift_ns() const noexcept {
    return dc_sync0_shift_ns_;
}

void SimBackend::set_sdo_write_abort(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::uint32_t abort_code) {
    if (slave < 1 || slave > slaves_.size()) {
        throw ConfigError("SimBackend::set_sdo_write_abort: slave " + std::to_string(slave) + " out of range (configured " +
                          std::to_string(slaves_.size()) + ")");
    }
    slaves_[slave - 1].sdo_write_aborts[sdo_key(index, sub)] = abort_code;
}

std::int64_t SimBackend::dc_time() const noexcept {
    return synthetic_dc_ns_;
}

std::int32_t SimBackend::received_profile_velocity(std::uint16_t slave) const noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        return slaves_[slave - 1].profile_velocity;
    }
    return 0;
}

std::int32_t SimBackend::received_target_velocity(std::uint16_t slave) const noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        return slaves_[slave - 1].target_velocity;
    }
    return 0;
}

std::int32_t SimBackend::velocity_at_qsa_exit(std::uint16_t slave) const noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        return slaves_[slave - 1].velocity_at_qsa_exit;
    }
    return 0;
}

bool SimBackend::entered_qsa(std::uint16_t slave) const noexcept {
    if (slave >= 1 && slave <= slaves_.size()) {
        return slaves_[slave - 1].entered_qsa;
    }
    return false;
}

std::vector<std::byte> SimBackend::recorded_sdo(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const {
    if (slave < 1 || slave > slaves_.size()) {
        return {};
    }
    const auto& dict = slaves_[slave - 1].dictionary;
    const auto it = dict.find(sdo_key(index, sub));
    return it == dict.end() ? std::vector<std::byte>{} : it->second;
}

std::vector<std::uint32_t> SimBackend::sdo_log(std::uint16_t slave) const {
    if (slave < 1 || slave > slaves_.size()) {
        return {};
    }
    return slaves_[slave - 1].sdo_write_order;
}

}  // namespace ethercat

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
    for (auto& model : slaves) {
        slaves_.emplace_back();
        slaves_.back().model = std::move(model);
    }
}

std::size_t SimBackend::open(std::string_view ifname) {
    if (open_) {
        throw Error("SimBackend::open: bus already open on '" + std::string(ifname) + "' (one master per backend; close() first)");
    }
    if (slaves_.empty()) {
        throw Error("SimBackend::open: no simulated slaves configured on '" + std::string(ifname) + "'");
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
        throw Error("SimBackend::slave_info: slave " + std::to_string(slave) + " out of range (1.." + std::to_string(slaves_.size()) + ")");
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
        throw Error("SimBackend::sdo_write: slave " + std::to_string(slave) + " out of range (configured " +
                    std::to_string(slaves_.size()) + ")");
    }
    Slave& s = slaves_[slave - 1];
    s.dictionary[sdo_key(index, sub)] = std::vector<std::byte>(data.begin(), data.end());
    s.sdo_write_order.push_back(sdo_key(index, sub));
}

std::size_t SimBackend::sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
    if (slave < 1 || slave > slaves_.size()) {
        throw Error("SimBackend::sdo_read: slave " + std::to_string(slave) + " out of range (configured " + std::to_string(slaves_.size()) +
                    ")");
    }
    const Slave& s = slaves_[slave - 1];
    // Conformant quick-stop configure gate: 0x605A (quick-stop option) reads 2 so a
    // PV/switchable configure() does not refuse; a written 0x6085 (decel) echoes back
    // via the dictionary below (nonzero -> the readback gate passes).
    if (index == 0x605A && sub == 0 && out.size() >= 2) {
        store_le<std::int16_t>(out.subspan(0, 2), 2);
        return 2;
    }
    if (const auto it = s.dictionary.find(sdo_key(index, sub)); it != s.dictionary.end()) {
        const std::size_t n = std::min(out.size(), it->second.size());
        for (std::size_t i = 0; i < n; ++i) {
            out[i] = it->second[i];
        }
        return n;
    }
    // Any other object reads as present-but-zero (a pipe, not a drive OD): zero-fill
    // so a steady-state SDO reader interleaving with a live move still gets bytes back.
    std::fill(out.begin(), out.end(), std::byte{0});
    return out.size();
}

void SimBackend::map_process_data() {
    expected_wkc_ = 0;
    for (auto& s : slaves_) {
        // Validate the required offsets fit the image sizes at SETUP time (step_device
        // is noexcept + uses subspan()), so a bad offset fails loudly here.
        const SimSlaveModel& m = s.model;
        if (m.ctrlword_off + 2 > m.output_bytes || m.target_off + 4 > m.output_bytes || m.statusword_off + 2 > m.input_bytes ||
            m.actual_off + 4 > m.input_bytes) {
            throw Error("SimSlaveModel offsets exceed the image sizes (out=" + std::to_string(m.output_bytes) +
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
        ++op_requests_;  // exactly one OP request per start
    }
    if (slave == 0) {
        for (auto& s : slaves_) {
            s.state = target;
        }
        return;
    }
    if (slave > slaves_.size()) {
        throw Error("SimBackend::request_state: slave " + std::to_string(slave) + " out of range (configured " +
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

    // Conformant instant mode echo: 0x6061 (mode-display) follows the 0x6060 the master
    // wrote. Lets the driver's enable-time mode gate (0x6061 == commanded) energize.
    if (s.model.mode_of_op_off >= 0 && s.model.mode_display_off >= 0) {
        const auto in = std::span<std::byte>(s.input_image);
        in[static_cast<std::size_t>(s.model.mode_display_off)] = out[static_cast<std::size_t>(s.model.mode_of_op_off)];
    }

    // CiA402 command decode (controlword masks).
    const bool shutdown = (cw & 0x87U) == 0x06U;
    const bool switch_on = (cw & 0x8FU) == 0x07U;
    const bool disable_voltage = (cw & 0x82U) == 0x00U;
    const bool enable_op = (cw & 0x8FU) == 0x0FU;
    const bool quick_stop = (cw & 0x86U) == 0x02U;

    using St = Cia402State;
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
            } else if (switch_on) {
                s.device_state = St::SwitchedOn;  // disable operation
            } else if (shutdown) {
                s.device_state = St::ReadyToSwitchOn;
            } else if (disable_voltage) {
                s.device_state = St::SwitchOnDisabled;
            }
            break;
        case St::QuickStopActive:
            // 0x605A=2 (decelerate, then auto-transition to SwitchOnDisabled): the stub has no
            // decel ramp, so it transitions to SwitchOnDisabled at once, modelling the real
            // drive's auto-disable that the policy's status-based teardown exit depends on.
            // enable_operation re-energizes.
            if (enable_op) {
                s.device_state = St::OperationEnabled;
            } else {
                s.device_state = St::SwitchOnDisabled;
            }
            break;
        case St::FaultReactionActive:
        case St::Fault:
            break;  // no fault modeling
    }

    // Profile-Position set-point-acknowledge handshake (bit4 -> bit12) + target latch.
    if (s.model.mode == Cia402Mode::ProfilePosition && s.device_state == St::OperationEnabled) {
        const bool bit4 = (cw & 0x10U) != 0U;
        const bool prev_bit4 = (prev & 0x10U) != 0U;
        if (bit4 && !prev_bit4) {
            s.target = load_le<std::int32_t>(out.subspan(s.model.target_off, 4));  // latch the new set-point
            s.setpoint_ack = true;
        } else if (!bit4 && prev_bit4) {
            s.setpoint_ack = false;
        }
    } else {
        s.setpoint_ack = false;
    }

    // Motion: chase the latched target (PP) or integrate the commanded velocity (PV).
    const std::int32_t actual_before = s.actual;
    if (s.device_state == St::OperationEnabled) {
        if (s.model.mode == Cia402Mode::ProfilePosition) {
            const std::int32_t step = s.model.counts_per_step;
            if (s.actual < s.target) {
                const std::int32_t next = static_cast<std::int32_t>(s.actual + step);
                s.actual = (next > s.target) ? s.target : next;
            } else if (s.actual > s.target) {
                const std::int32_t next = static_cast<std::int32_t>(s.actual - step);
                s.actual = (next < s.target) ? s.target : next;
            }
        } else if (s.model.velocity_off >= 0) {
            const std::int32_t v = load_le<std::int32_t>(out.subspan(static_cast<std::size_t>(s.model.velocity_off), 4));
            s.actual = static_cast<std::int32_t>(s.actual + v);
        }
    }
    s.velocity = static_cast<std::int32_t>(s.actual - actual_before);  // per-cycle delta

    // Compose the statusword.
    unsigned sw = statusword_base(s.device_state);
    sw |= 0x0200U;  // bit9 remote
    if (s.device_state != St::SwitchOnDisabled && s.device_state != St::NotReadyToSwitchOn) {
        sw |= 0x0010U;  // bit4 voltage enabled
    }
    if (s.setpoint_ack) {
        sw |= 0x1000U;  // bit12 set-point acknowledge
    }
    if (s.device_state == St::OperationEnabled && s.model.mode == Cia402Mode::ProfilePosition && s.actual == s.target) {
        sw |= 0x0400U;  // bit10 target reached (conformant: set only at target)
    }

    const auto in = std::span<std::byte>(s.input_image);
    store_le<std::uint16_t>(in.subspan(s.model.statusword_off, 2), static_cast<std::uint16_t>(sw));
    store_le<std::int32_t>(in.subspan(s.model.actual_off, 4), s.actual);
    if (s.model.velocity_actual_off >= 0) {
        store_le<std::int32_t>(in.subspan(static_cast<std::size_t>(s.model.velocity_actual_off), 4), s.velocity);
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

void SimBackend::force_short_wkc_once() noexcept {
    short_wkc_once_ = true;
}

void SimBackend::force_short_wkc(bool on) noexcept {
    short_wkc_sticky_.store(on, std::memory_order_relaxed);
}

void SimBackend::arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) {
    dc_cycle_ns_ = cycle_ns;              // record that the bring-up armed SYNC0 (configured_dc_cycle_ns)
    dc_sync0_shift_ns_ = sync0_shift_ns;  // record the CyclShift the config threaded through
}

std::uint32_t SimBackend::configured_dc_cycle_ns() const noexcept {
    return dc_cycle_ns_;
}

std::int32_t SimBackend::configured_dc_sync0_shift_ns() const noexcept {
    return dc_sync0_shift_ns_;
}

std::int64_t SimBackend::dc_time() const noexcept {
    return synthetic_dc_ns_;
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

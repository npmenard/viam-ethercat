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
    slaves_.reserve(slaves.size());
    for (auto& model : slaves) {
        Slave slave;
        slave.model = std::move(model);
        slaves_.push_back(std::move(slave));
    }
}

std::size_t SimBackend::scan(std::string_view ifname) {
    if (open_) {
        throw ConfigError("SimBackend::scan: bus already open on '" + std::string(ifname) + "' (one master per backend; close() first)");
    }
    if (slaves_.empty()) {
        throw InitError("SimBackend::scan: no simulated slaves configured on '" + std::string(ifname) + "'");
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
        throw PdoMappingError("SimBackend::sdo_write: slave " + std::to_string(slave) + " out of range");
    }
    Slave& s = slaves_[slave - 1];
    s.dictionary[sdo_key(index, sub)] = std::vector<std::byte>(data.begin(), data.end());
    s.sdo_write_order.push_back(sdo_key(index, sub));
}

std::size_t SimBackend::sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
    if (slave < 1 || slave > slaves_.size()) {
        throw PdoMappingError("SimBackend::sdo_read: slave " + std::to_string(slave) + " out of range");
    }
    const Slave& s = slaves_[slave - 1];
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

void SimBackend::map_process_image() {
    expected_wkc_ = 0;
    for (auto& s : slaves_) {
        s.output_image.assign(s.model.output_bytes, std::byte{0});
        s.input_image.assign(s.model.input_bytes, std::byte{0});
        // Each slave with both a command and feedback image contributes 3 to the
        // working counter (2 write + 1 read), mirroring SOEM's accounting.
        expected_wkc_ += (s.model.output_bytes > 0 ? 2 : 0) + (s.model.input_bytes > 0 ? 1 : 0);
        s.state = EcatState::SafeOp;
    }
}

void SimBackend::request_state(std::uint16_t slave, EcatState target) {
    if (slave == 0) {
        for (auto& s : slaves_) {
            s.state = target;
        }
        return;
    }
    if (slave > slaves_.size()) {
        throw InitError("SimBackend::request_state: slave " + std::to_string(slave) + " out of range");
    }
    slaves_[slave - 1].state = target;
}

EcatState SimBackend::state(std::uint16_t slave) const {
    if (slave == 0) {
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

std::span<std::byte> SimBackend::outputs(std::uint16_t slave) noexcept {
    if (slave < 1 || slave > slaves_.size()) {
        return {};
    }
    return slaves_[slave - 1].output_image;
}

std::span<const std::byte> SimBackend::inputs(std::uint16_t slave) const noexcept {
    if (slave < 1 || slave > slaves_.size()) {
        return {};
    }
    return slaves_[slave - 1].input_image;
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

    using St = Cia402State;
    if (s.faulted && s.device_state != St::Fault) {
        s.device_state = St::Fault;
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
                s.device_state = St::SwitchOnDisabled;
            } else if (enable_op) {
                s.device_state = St::OperationEnabled;
            }
            break;
        case St::FaultReactionActive:
            s.device_state = St::Fault;
            break;
        case St::Fault:
            if (fault_reset_rising) {
                s.faulted = false;
                s.device_state = St::SwitchOnDisabled;
            }
            break;
    }

    // Profile-Position set-point-acknowledge handshake (bit4 / bit12).
    if (s.model.mode == Cia402Mode::ProfilePosition && s.device_state == St::OperationEnabled) {
        const bool bit4 = (cw & 0x10U) != 0U;
        const bool prev_bit4 = (prev & 0x10U) != 0U;
        if (bit4 && !prev_bit4) {
            s.target = load_le<std::int32_t>(out.subspan(s.model.target_off, 4));
            s.setpoint_ack = true;
        } else if (!bit4 && prev_bit4) {
            s.setpoint_ack = false;
        }
    } else {
        s.setpoint_ack = false;
    }

    // Motion: chase the target (PP) or integrate velocity (PV).
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
        } else if (s.model.mode == Cia402Mode::ProfileVelocity && s.model.velocity_off >= 0) {
            const std::int32_t vel = load_le<std::int32_t>(out.subspan(static_cast<std::size_t>(s.model.velocity_off), 4));
            s.actual = static_cast<std::int32_t>(s.actual + vel);
        }
    }

    // Compose the statusword.
    unsigned sw = statusword_base(s.device_state);
    sw |= 0x0200U;  // bit9 remote
    sw |= 0x0400U;  // bit10 ALWAYS 1 -- A6 quirk (never usable for move-complete)
    if (s.device_state != St::SwitchOnDisabled && s.device_state != St::NotReadyToSwitchOn) {
        sw |= 0x0010U;  // bit4 voltage enabled
    }
    if (s.setpoint_ack) {
        sw |= 0x1000U;  // bit12 set-point acknowledge
    }

    const auto in = std::span<std::byte>(s.input_image);
    store_le<std::uint16_t>(in.subspan(s.model.statusword_off, 2), static_cast<std::uint16_t>(sw));
    store_le<std::int32_t>(in.subspan(s.model.actual_off, 4), s.actual);

    s.prev_ctrlword = cw;
}

int SimBackend::send_receive() noexcept {
    if (!open_) {
        return -1;
    }
    for (auto& s : slaves_) {
        // Only in OP are the RxPDO outputs live, so only then does the device
        // consume the controlword and advance its CiA402 state.
        if (s.state == EcatState::Op) {
            step_device(s);
        }
    }
    if (short_wkc_once_) {
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
        slaves_[slave - 1].faulted = true;
    }
}

void SimBackend::force_short_wkc_once() noexcept {
    short_wkc_once_ = true;
}

std::vector<std::byte> SimBackend::recorded_sdo(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const {
    if (slave < 1 || slave > slaves_.size()) {
        return {};
    }
    const auto& dict = slaves_[slave - 1].dictionary;
    const auto it = dict.find(sdo_key(index, sub));
    return it == dict.end() ? std::vector<std::byte>{} : it->second;
}

}  // namespace ethercat

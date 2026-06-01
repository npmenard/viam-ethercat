#include "ethercat/master.hpp"

#include <string>
#include <utility>

namespace ethercat {

namespace {

constexpr int kPrimeCycles = 3;  // exchanges in SAFE-OP so slaves have valid outputs before OP

std::uint32_t field_key(std::uint16_t index, std::uint8_t sub) noexcept {
    return (static_cast<std::uint32_t>(index) << 8U) | sub;
}

}  // namespace

Master::Master(MasterConfig config, std::unique_ptr<EcatBackend> backend) : config_(std::move(config)), backend_(std::move(backend)) {
    if (!backend_) {
        throw ConfigError("Master: null backend");
    }
    if (config_.ifname.empty()) {
        throw ConfigError("Master: empty interface name");
    }
    if (config_.slaves.empty()) {
        throw ConfigError("Master: no slaves configured");
    }
    if (config_.target_loop_rate_hz == 0 || config_.target_loop_rate_hz > 1000) {
        throw ConfigError("Master: target_loop_rate_hz " + std::to_string(config_.target_loop_rate_hz) + " out of range (1..1000)");
    }
}

void Master::init() {
    const std::size_t count = backend_->open(config_.ifname);
    if (count != config_.slaves.size()) {
        throw InitError("EtherCAT bus on '" + config_.ifname + "': found " + std::to_string(count) + " slaves, config expects " +
                        std::to_string(config_.slaves.size()));
    }
}

std::map<std::uint32_t, FieldLocation> Master::build_field_table(std::uint16_t slave, const PdoMap& map) {
    std::map<std::uint32_t, FieldLocation> fields;
    std::size_t bit = 0;
    for (const std::uint16_t pdo : map.pdo_indices) {
        const auto it = map.entries.find(pdo);
        if (it == map.entries.end()) {
            throw PdoMappingError("slave " + std::to_string(slave) + ": PDO has no entry list while building field table");
        }
        for (const PdoEntry& e : it->second) {
            if (e.index != 0x0000) {  // 0x0000 = padding/gap: advances the offset, no named field
                if ((bit % 8) != 0) {
                    throw PdoMappingError("slave " + std::to_string(slave) + ": mapped object is not byte-aligned (bit offset " +
                                          std::to_string(bit) + ")");
                }
                if ((e.bit_length % 8) != 0) {
                    throw PdoMappingError("slave " + std::to_string(slave) + ": mapped object width " + std::to_string(e.bit_length) +
                                          " bits is not a whole number of bytes");
                }
                fields[field_key(e.index, e.subindex)] = FieldLocation{bit / 8, e.bit_length / 8U};
            }
            bit += e.bit_length;
        }
    }
    return fields;
}

void Master::configure() {
    // Maps are writable only in PRE-OP and are not stored in EEPROM, so this runs
    // every configure() / power-on.
    backend_->request_state(0, EcatState::PreOp);

    for (const SlaveConfig& sc : config_.slaves) {
        apply_pdo_map(*backend_, sc.slave_id, sc.rxpdo);
        apply_pdo_map(*backend_, sc.slave_id, sc.txpdo);
    }

    backend_->map_process_data();
    expected_wkc_ = backend_->expected_wkc();

    slaves_.clear();
    for (const SlaveConfig& sc : config_.slaves) {
        const SlaveInfo info = backend_->slave_info(sc.slave_id);

        // Validate the APPLIED (wire) image against the CONFIGURED map. If a real
        // drive silently rejected part of the remap, map_process_data lays out the
        // drive's default image while the field table (built from config below)
        // carries offsets for the expected map -- a store_le into outputs at a
        // config-derived offset could then run past the wire-sized span (OOB in
        // the noexcept RT loop). Fail loudly here instead.
        const std::size_t rx_bytes = sc.rxpdo.byte_size();
        const std::size_t tx_bytes = sc.txpdo.byte_size();
        if (info.output_bytes != rx_bytes || info.input_bytes != tx_bytes) {
            throw PdoMappingError("slave " + std::to_string(sc.slave_id) + ": applied RxPDO " + std::to_string(info.output_bytes) +
                                  " B / TxPDO " + std::to_string(info.input_bytes) + " B != configured " + std::to_string(rx_bytes) +
                                  " / " + std::to_string(tx_bytes) + " B (remap did not take)");
        }

        // PdoCache(rx = FEEDBACK size (TxPDO/inputs), tx = COMMAND size (RxPDO/outputs)).
        slaves_.emplace_back(sc.slave_id, info.input_bytes, info.output_bytes);
        SlaveRuntime& rt = slaves_.back();
        rt.io = backend_->slave_io(sc.slave_id);
        rt.rx_fields = build_field_table(sc.slave_id, sc.rxpdo);
        rt.tx_fields = build_field_table(sc.slave_id, sc.txpdo);
    }

    backend_->request_state(0, EcatState::SafeOp);
    for (int i = 0; i < kPrimeCycles; ++i) {
        (void)backend_->exchange();
    }
    backend_->request_state(0, EcatState::Op);
    if (backend_->slave_state(0) != EcatState::Op) {
        throw InitError("EtherCAT bus on '" + config_.ifname + "': not all slaves reached OPERATIONAL (bus is " +
                        to_string(backend_->slave_state(0)) + ")");
    }

    fault_.store(false, std::memory_order_relaxed);
    consecutive_wkc_errors_ = 0;
    operational_.store(true, std::memory_order_relaxed);
}

void Master::process() noexcept {
    const int wkc = backend_->exchange();
    if (wkc < 0 || wkc < expected_wkc_) {
        ++consecutive_wkc_errors_;
        if (consecutive_wkc_errors_ >= config_.max_consecutive_wkc_errors) {
            // Store the payload (fault_wkc_) first, then publish the flag with a
            // release store so a reader that acquires fault_==true sees the wkc.
            fault_wkc_.store(wkc, std::memory_order_relaxed);
            fault_.store(true, std::memory_order_release);
            operational_.store(false, std::memory_order_relaxed);
        }
    } else {
        consecutive_wkc_errors_ = 0;
        working_counter_.store(wkc, std::memory_order_relaxed);
    }

    ++cycle_;
    for (SlaveRuntime& rt : slaves_) {
        rt.cache.publish_inputs(rt.io.inputs, static_cast<std::uint16_t>(wkc < 0 ? 0 : wkc), cycle_);
    }
}

void Master::close() noexcept {
    operational_.store(false, std::memory_order_relaxed);
    backend_->close();
}

std::span<std::byte> Master::outputs(std::uint16_t slave) noexcept {
    // rt can be const: SlaveIo::outputs is a std::span<std::byte> (shallow-const),
    // so a const SlaveRuntime still yields a writable view of the command image.
    for (const SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt.io.outputs;
        }
    }
    return {};
}

PdoSnapshot Master::read_inputs(std::uint16_t slave) const noexcept {
    for (const SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt.cache.read_inputs();
        }
    }
    return {};
}

Master::SlaveRuntime& Master::runtime_for(std::uint16_t slave) {
    for (SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt;
        }
    }
    throw ConfigError("Master: unknown slave " + std::to_string(slave));
}

const Master::SlaveRuntime& Master::runtime_for(std::uint16_t slave) const {
    for (const SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt;
        }
    }
    throw ConfigError("Master: unknown slave " + std::to_string(slave));
}

PdoCache& Master::cache(std::uint16_t slave) {
    return runtime_for(slave).cache;
}

FieldLocation Master::rx_field(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const {
    const SlaveRuntime& rt = runtime_for(slave);
    const auto it = rt.rx_fields.find(field_key(index, sub));
    if (it == rt.rx_fields.end()) {
        throw PdoMappingError("slave " + std::to_string(slave) + ": object not in the RxPDO (command) map");
    }
    return it->second;
}

FieldLocation Master::tx_field(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const {
    const SlaveRuntime& rt = runtime_for(slave);
    const auto it = rt.tx_fields.find(field_key(index, sub));
    if (it == rt.tx_fields.end()) {
        throw PdoMappingError("slave " + std::to_string(slave) + ": object not in the TxPDO (feedback) map");
    }
    return it->second;
}

std::string Master::last_error() const {
    // Acquire pairs with process()'s release store of fault_, so fault_wkc_ below
    // is the value that was current when the fault latched (never stale).
    if (!fault_.load(std::memory_order_acquire)) {
        return {};
    }
    return "EtherCAT working-counter fault on '" + config_.ifname + "': got " + std::to_string(fault_wkc_.load(std::memory_order_relaxed)) +
           ", expected " + std::to_string(expected_wkc_) + " for " + std::to_string(config_.max_consecutive_wkc_errors) +
           " consecutive cycles";
}

}  // namespace ethercat

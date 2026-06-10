// Entrypoint for the Viam EtherCAT servo motor module.
//
// Mirrors the reference yaskawa/UR module main: an Instance, a boost::asio
// io_context running on its own thread, and a ModuleService serving the
// viam:ethercat:servo (rdk:component:motor) registration. The motor's RT work
// lives in ServoController's own jthread; the io_context backs the SDK's async
// gRPC serving.

#include <cstdlib>
#include <exception>
#include <memory>
#include <thread>

#include <boost/asio.hpp>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/service.hpp>

#include "viam/module/servo_motor.hpp"

using namespace viam::sdk;

int main(int argc, char** argv) {
    try {
        const Instance instance;

        boost::asio::io_context io_context;
        std::thread io_thread([&io_context] {
            try {
                const boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard(io_context.get_executor());
                io_context.run();
            } catch (const std::exception& ex) {
                VIAM_SDK_LOG(error) << "error in io_thread: " << ex.what();
            }
        });

        std::make_shared<ModuleService>(argc, argv, ethercat::servo::ServoMotor::create_model_registrations())->serve();

        io_context.stop();
        io_thread.join();
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        VIAM_SDK_LOG(error) << "error in module: " << ex.what();
        return EXIT_FAILURE;
    }
}

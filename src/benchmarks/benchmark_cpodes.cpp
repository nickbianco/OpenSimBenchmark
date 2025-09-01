#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Benchmark a forward simulation using the CPodes integrator.

Usage:
  benchmark_cpodes <model> <output> --time=<time> --step=<step> --accuracy=<accuracy>
  benchmark_cpodes -h | --help

Options:
  -t <time>,      --time <time>            Set the final time.
  -s <step>,      --step <step>            Set the step size.
  -a <accuracy>,  --accuracy <accuracy>    Set the integrator accuracy.
)";

int main(int argc, char** argv) {

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::microseconds;

    // Disable logging.
    OpenSim::Logger::setLevelString("error");

    // Parse the command line arguments.
    std::map<std::string, docopt::value> args = parse_arguments(
        HELP, { argv + 1, argv + argc },
        true); // show help if requested

    // Model.
    OPENSIM_THROW_IF(!args["<model>"], OpenSim::Exception,
        "Model file must be provided.");
    OpenSim::Model model(args["<model>"].asString());
    model.initSystem();

    // Final time.
    double time = -1.0; // seconds
    if (args["--time"]) {
        time = std::stod(args["--time"].asString());
        OPENSIM_THROW_IF(time <= 0, OpenSim::Exception,
                "Final time must be positive.");
    }

    // Step size.
    double step = -1.0; // seconds
    if (args["--step"]) {
        step = std::stod(args["--step"].asString());
    }

    // Accuracy.
    double accuracy = -1.0;
    if (args["--accuracy"]) {
        accuracy = std::stod(args["--accuracy"].asString());
        OPENSIM_THROW_IF(accuracy <= 0, OpenSim::Exception,
                "Accuracy must be positive.");
    }

    // Get the output file.
    std::string output_file = args["<output>"].asString();

    // Initialize the JSON object.
    json j;

    // Run benchmarks.
    SimTK::Vector results = run_benchmarks(model, time, step, accuracy,
            Manager::IntegratorMethod::CPodes);
    j["acceleration_compute_time"] = results[0];
    j["single_step_time"] = results[1];
    j["forward_integration_time"] = results[2];
    j["real_time_factor"] = results[3];
    j["energy_conservation"] = results[4];
    j["num_steps"] = results[5];
    j["time_per_step"] = results[6];

    std::ofstream file(output_file, std::ios::out | std::ios::binary);
    if (file) {
        file << j.dump(4);
    }
}
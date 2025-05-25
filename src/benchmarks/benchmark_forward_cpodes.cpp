#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Benchmark a forward simulation with contact using the CPodes integrator.

Usage:
  benchmark_forward_cpodes <model> <output> --time=<time> --step=<step> --accuracy=<accuracy> --parameter=<parameter> --scale=<scale>
  benchmark_forward_cpodes -h | --help

Options:
  -t <time>,      --time <time>            Set the final time.
  -s <step>,      --step <step>            Set the step size.
  -a <accuracy>,  --accuracy <accuracy>    Set the integrator accuracy.
  -p <parameter>, --parameter <parameter>  Set the contact parameter to evaluate.
  -s <scale>,     --scale <scale>          Scale factor for the parameter.
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

    // Parameter.
    std::string parameter = "none";
    if (args["--parameter"]) {
        parameter = args["--parameter"].asString();
        OPENSIM_THROW_IF(parameter != "stiffness" && 
                        parameter != "dissipation" &&
                        parameter != "friction" &&
                        parameter != "smoothing",
                        OpenSim::Exception,
                        "Invalid parameter specified. Must be one of: "
                        "'stiffness', 'dissipation', 'friction', 'smoothing'.");
    }

    // Scale factor.
    double scale = 1.0; // default scale
    if (args["--scale"]) {
        scale = std::stod(args["--scale"].asString());
        OPENSIM_THROW_IF(scale <= 0, OpenSim::Exception, 
                "Scale factor must be positive.");
    }

    // Scale the specified parameter.
    for (auto& force : model.updComponentList<OpenSim::SmoothSphereHalfSpaceForce>()) {
        if (parameter == "stiffness") {
            force.set_stiffness(force.get_stiffness() * scale);
        } else if (parameter == "dissipation") {
            force.set_dissipation(force.get_dissipation() * scale);
        } else if (parameter == "friction") {
            force.set_dynamic_friction(scale * force.get_dynamic_friction());
            force.set_static_friction(scale * force.get_static_friction());
            force.set_viscous_friction(scale * force.get_viscous_friction());
        } else if (parameter == "smoothing") {
            force.set_hertz_smoothing(scale * force.get_hertz_smoothing());
            force.set_hunt_crossley_smoothing(scale * force.get_hunt_crossley_smoothing());
        } else {
            OPENSIM_THROW(OpenSim::Exception, 
                "Invalid parameter specified. Must be one of: "
                "'stiffness', 'dissipation', 'friction', 'smoothing'.");
        }
    }

    // Initialize the system.
    SimTK::State state = model.initSystem();
    const SimTK::MultibodySystem& system = model.getMultibodySystem();

    // Helper function to reset the state.
    Vector defaultY = state.getY();
    auto resetState = [&](SimTK::State& state, bool randomize = false) {
        state.setTime(0);
        state.updY() = defaultY;
        if (randomize) {
            state.updY() += 1e-6 * SimTK::Test::randVector(state.getNY());
        }
    };

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

    // Integrator accuracy.
    double accuracy = -1.0;
    if (args["--accuracy"]) {
        accuracy = std::stod(args["--accuracy"].asString());
        OPENSIM_THROW_IF(accuracy <= 0, OpenSim::Exception, 
                "Integrator accuracy must be positive.");
    }

    // Get the output file.
    std::string output_file = args["<output>"].asString();

    // Initialize the JSON object.
    json j; 

    // Forward integration.
    // --------------------
    int num_reps = 10;
    SimTK::Vector integration_times(num_reps, 0.0);
    SimTK::Vector real_time_factors(num_reps, 0.0);
    std::vector<int> num_steps(num_reps);
    num_steps.reserve(num_reps);
    SimTK::Vector average_step_sizes(num_reps, 0.0);
    SimTK::Vector times_per_step(num_reps, 0.0);
    for (int i = 0; i < num_reps; ++i) {
        SimTK::CPodesIntegrator integrator(system,
            SimTK::CPodes::BDF, SimTK::CPodes::Newton);
        if (step > 0) {
            integrator.setFixedStepSize(step);
        }
        integrator.setAccuracy(accuracy);
        SimTK::TimeStepper timeStepper(system, integrator);
        resetState(state, true);
        integrator.resetAllStatistics();
        timeStepper.initialize(state);
        auto start = high_resolution_clock::now();
        timeStepper.stepTo(time);
        auto end = high_resolution_clock::now();
        double time_elapsed = duration_cast<microseconds>(end - start).count();
        time_elapsed /= 1.0e6;
        integration_times[i] = time_elapsed;
        real_time_factors[i] = time / time_elapsed;
        num_steps.push_back(integrator.getNumStepsTaken());
        average_step_sizes[i] = (1000.0 * time) / integrator.getNumStepsTaken();
        times_per_step[i] = 1000.0 * time_elapsed / integrator.getNumStepsTaken();
    }
    j["forward_integration_time"] = SimTK::mean(integration_times);
    j["real_time_factor"] = SimTK::mean(real_time_factors);
    j["num_steps"] = std::accumulate(num_steps.begin(), num_steps.end(), 0) / num_reps;
    j["average_step_size"] = SimTK::mean(average_step_sizes);
    j["time_per_step"] = SimTK::mean(times_per_step);

    std::ofstream file(output_file, std::ios::out | std::ios::binary);
    if (file) {
        file << j.dump(4);
    }
}
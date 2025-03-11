#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Analyze a forward simulation with 'perf'.

Usage:
  perf_forward <model> <output> [--time <time>]
  perf_forward -h | --help

Options:
  -t <time>, --time <time>        Set the final time.
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
    SimTK::State state = model.initSystem();

    // Final time and step size.
    double time = 1.0; // seconds
    if (args["--time"]) {
        time = std::stod(args["--time"].asString());
        OPENSIM_THROW_IF(time <= 0, OpenSim::Exception, 
                "Final time must be positive.");
    }

    double step = -1; // seconds
    if (args["--step"]) {
        step = std::stod(args["--step"].asString());
        OPENSIM_THROW_IF(step <= 0, OpenSim::Exception, 
                "Step size must be positive.");
    }

    // Get the output file.
    std::string output_file = args["<output>"].asString();

    // Run the forward integration simulation.
    state.setTime(0);

    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    SimTK::TimeStepper timeStepper(model.getMultibodySystem(), integrator);
    timeStepper.initialize(state);
    
    auto start = high_resolution_clock::now();
    timeStepper.stepTo(time);
    auto end = high_resolution_clock::now();
    double time_elapsed = duration_cast<microseconds>(end - start).count();
    // Convert to seconds.
    time_elapsed /= 1e6;

    // Save the results.
    json j;
    j["time_elapsed"] = time_elapsed;
    j["time"] = time;
    j["num_steps_attempted"] = integrator.getNumStepsAttempted();
    j["num_steps_taken"] = integrator.getNumStepsTaken();
    j["num_realizations"] = integrator.getNumRealizations();
    j["num_q_projections"] = integrator.getNumQProjections();
    j["num_u_projections"] = integrator.getNumUProjections();
    j["num_projections"] = integrator.getNumProjections();
    j["num_error_test_failures"] = integrator.getNumErrorTestFailures();
    j["num_convergence_test_failures"] = 
            integrator.getNumConvergenceTestFailures();
    j["num_iterations"] = integrator.getNumIterations();
    j["num_realization_failures"] = integrator.getNumRealizationFailures();
    j["num_q_projection_failures"] = integrator.getNumQProjectionFailures();
    j["num_u_projection_failures"] = integrator.getNumUProjectionFailures();
    j["num_projection_failures"] = integrator.getNumProjectionFailures();
    j["num_convergent_iterations"] = integrator.getNumConvergentIterations();
    j["num_divergent_iterations"] = integrator.getNumDivergentIterations();
    j["initial_step_size"] = integrator.getActualInitialStepSizeTaken();
    j["final_step_size"] = integrator.getPreviousStepSizeTaken();
    j["time"] = time;
    j["time_elapsed"] = time_elapsed;
    j["time_per_realization"] = time_elapsed / integrator.getNumRealizations();

    std::ofstream file(output_file, std::ios::out | std::ios::binary);
    if (file) {
        file << j.dump(4);
    }
}
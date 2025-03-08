#include <OpenSim/OpenSim.h>
#include <string>
#include "../utilities/utilities.h"

static const char HELP[] =
R"(Analyze a forward simulation with 'perf'.

Usage:
  perf_forward <model> <output> [--time <time>]
  perf_forward -h | --help

Options:
  -t <time>, --time <time>        Set the final time.
)";


int main(int argc, char** argv) {
    
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
    
    timeStepper.stepTo(time);

    std::unordered_map<std::string, int> outputs;
    outputs["num_steps_attempted"] = integrator.getNumStepsAttempted();
    outputs["num_steps_taken"] = integrator.getNumStepsTaken();
    outputs["num_realizations"] = integrator.getNumRealizations();
    outputs["num_q_projections"] = integrator.getNumQProjections();
    outputs["num_u_projections"] = integrator.getNumUProjections();
    outputs["num_projections"] = integrator.getNumProjections();
    outputs["num_error_test_failures"] = integrator.getNumErrorTestFailures();
    outputs["num_convergence_test_failures"] = 
            integrator.getNumConvergenceTestFailures();
    outputs["num_iterations"] = integrator.getNumIterations();
    outputs["num_realization_failures"] = 
            integrator.getNumRealizationFailures();
    outputs["num_q_projection_failures"] = 
            integrator.getNumQProjectionFailures();
    outputs["num_u_projection_failures"] = 
            integrator.getNumUProjectionFailures();
    outputs["num_projection_failures"] = 
            integrator.getNumProjectionFailures();
    outputs["num_convergent_iterations"] = 
            integrator.getNumConvergentIterations();
    outputs["num_divergent_iterations"] = 
            integrator.getNumDivergentIterations();

    saveMapToJsonFile(outputs, output_file);
}
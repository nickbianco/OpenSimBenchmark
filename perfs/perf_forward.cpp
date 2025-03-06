#include <OpenSim/OpenSim.h>
#include <string>
#include "../utilities/utilities.h"

static const char HELP[] =
R"(Analyze a forward simulation with 'perf'.

Usage:
  perf_forward <model> [--time <time>] [--step <step>] [--randomize <bool>]
  perf_forward -h | --help

Options:
  -t <time>, --time <time>        Set the final time.
  -s <step>, --step <step>        Set the step size.
  -r <bool>, --randomize <bool>   Randomize the initial state.
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

    bool randomize = false;
    if (args["--randomize"]) {
        randomize = (bool)std::stod(args["--randomize"].asString());
        std::cout << "Randomize initial state: " << randomize << std::endl;
    }

    // Create a state generator.
    StateGenerator generator(model);

    if (randomize) {
        // Get a random state vector.
        state.updY() = generator.getRandomY();

        // Get a random control vector.
        SimTK::Vector controls = generator.getRandomControls();
        model.realizeVelocity(state);
        model.setControls(state, controls);  
    }

    state.setTime(0);

    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    if (step > 0) {
        integrator.setFixedStepSize(step);
    }

    SimTK::TimeStepper timeStepper(model.getMultibodySystem(), integrator);
    timeStepper.initialize(state);
    
    timeStepper.stepTo(time);
}
#include <OpenSim/OpenSim.h>
#include <string>
#include "../utilities/utilities.h"

// static const char HELP[] =
// R"(Benchmark realizing an OpenSim::Model to a SimTK::Stage.

// Usage:
//   perf_realize <model>
//   perf_realize -h | --help
// )";

int main(int argc, char** argv) {
    
    // Disable logging.
    OpenSim::Logger::setLevelString("error");

    // Parse the command line arguments.
    // std::map<std::string, docopt::value> args = parse_arguments(
    //     HELP, { argv + 1, argv + argc - 1 },
    //     true); // show help if requested

    // Model.
    // OPENSIM_THROW_IF(!args["<model>"], OpenSim::Exception, 
    //         "Model file must be provided.");
    OpenSim::Model model(argv[1]);
    SimTK::State state = model.initSystem();

    model.getSystem().realize(state, SimTK::Stage::Acceleration);
}
#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Analyze realizing an OpenSim::Model to SimTK::Stage::Acceleration with 'perf'.

Usage:
  perf_realize <model> <output>
  perf_realize -h | --help
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

     // Output file.
     std::string output_file = args["<output>"].asString();

    // Realize to SimTK::Stage::Acceleration.
    auto start = high_resolution_clock::now();
    model.realizeAcceleration(state);
    auto end = high_resolution_clock::now();
    double time_elapsed = duration_cast<microseconds>(end - start).count();
    // Convert to seconds.
    time_elapsed /= 1e6;

    // Save the results.
    json j;
    j["time_elapsed"] = time_elapsed;
    std::ofstream file(output_file, std::ios::out | std::ios::binary);
    if (file) {
        file << j.dump(4);
    }
}
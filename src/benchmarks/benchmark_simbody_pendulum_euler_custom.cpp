#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Benchmark a simulation of an N-link pendulum in Simbody.

Usage:
  benchmark_simbody_pendulum_euler_custom <nlinks> <output> --time=<time> --step=<step>
  benchmark_simbody_pendulum_euler_custom -h | --help

Options:
  -t <time>, --time <time>        Set the final time.
  -s <step>, --step <step>        Set the step size.
)";

void resetState(SimTK::State& state) {
    // Reset the state to the initial state.
    state.setTime(0);
    state.updQ() = SimTK::Vector(state.getNQ(), 0.0);
    state.updQ()[0] = SimTK::Pi / 4.0;
    state.updU() = SimTK::Vector(state.getNU(), 0.0);
}

class MyIntegrator {

public:
    MyIntegrator(const System& system, const State& initialState) : system(system) {
        advancedState = system.realizeTopology();
        advancedState.setTime(initialState.getTime());
        advancedState.updY() = initialState.getY();
        setPreviousStateFromAdvancedState();
    }

    const System& getSystem() const {
        return system;
    }

    void setAdvancedState(const Real& t, const Vector& q, const Vector& u,
            const Vector& z) {
        advancedState.updQ() = q;
        advancedState.updU() = u;
        advancedState.updZ() = z;
        advancedState.updTime() = t;
    }

    const State& getAdvancedState() const { return advancedState; }
    Real getAdvancedTime() const { return advancedState.getTime(); }

    const Real& getPreviousTime() const { return tPrev; }
    const Vector& getPreviousQ() const { return qPrev; }
    const Vector& getPreviousQDot() const { return qDotPrev; }
    const Vector& getPreviousU() const { return uPrev; }    
    const Vector& getPreviousUDot() const { return uDotPrev; }
    const Vector& getPreviousZ() const { return zPrev; }
    const Vector& getPreviousZDot() const { return zDotPrev; }

    void realizeStateDerivatives(const SimTK::State& state) {
        if (state.getSystemStage() < Stage::Acceleration) {
            getSystem().realize(state, Stage::Acceleration);
        }       
    }

    void setAdvancedStateAndRealizeDerivatives(const Real& t, const Vector& q,
            const Vector& u, const Vector& z) {
        setAdvancedState(t, q, u, z);
        realizeStateDerivatives(getAdvancedState());
    }

    void setPreviousStateFromAdvancedState() {
        tPrev = advancedState.getTime();
        qPrev = advancedState.getQ();
        qDotPrev = advancedState.getQDot();
        uPrev = advancedState.getU();
        uDotPrev = advancedState.getUDot();
        zPrev = advancedState.getZ();
        zDotPrev = advancedState.getZDot();
    }

    void takeOneStep(Real h) {
        takeOneEulerStep(h);
        setPreviousStateFromAdvancedState();
    }

    // Semi-explicit Euler.
    void takeOneEulerStep(Real h) {
        const Real t0 = getPreviousTime();
        const Real t1 = t0 + h;

        advancedState.updZ() = getPreviousZ() + h * getPreviousZDot();
        advancedState.updU() = getPreviousU() + h * getPreviousUDot();
        advancedState.updQ() = getPreviousQ() + h * advancedState.getU();
        getSystem().realize(advancedState, Stage::Velocity);
        // realizeStateDerivatives(advancedState);
    }

private:
    const System& system;
    State advancedState;
    Real tPrev;
    Vector qPrev;
    Vector qDotPrev;
    Vector uPrev;
    Vector uDotPrev;
    Vector zPrev;
    Vector zDotPrev;
};

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

    // Pendulum system.
    int numLinks = std::stoi(args["<nlinks>"].asString());
    OPENSIM_THROW_IF(numLinks <= 0, OpenSim::Exception, 
            "Number of links must be positive.");
    PendulumSystem pendulum(numLinks);

    // Final time.
    double time = 5.0; // seconds
    if (args["--time"]) {
        time = std::stod(args["--time"].asString());
        OPENSIM_THROW_IF(time <= 0, OpenSim::Exception, 
                "Final time must be positive.");
    }

    // Step size.
    double step = -1; // seconds
    if (args["--step"]) {
        step = std::stod(args["--step"].asString());
    }

    // Get the output file.
    std::string output_file = args["<output>"].asString();

    // Initialize the JSON object.
    json j;

    // Create the initial state.
    const SimTK::MultibodySystem& system = pendulum.m_system;
    SimTK::State state = system.realizeTopology();

    // Initial energy
    // --------------
    resetState(state);
    system.realize(state, SimTK::Stage::Dynamics);
    double initial_energy = system.calcEnergy(state);

    // Forward integration.
    // --------------------
    resetState(state);
    MyIntegrator integrator(system, state);
    SimTK::Vector final_energies(10, 0.0);
    SimTK::Vector integration_times(10, 0.0);
    SimTK::Vector real_time_factors(10, 0.0);
    int numSteps = int(time / step);
    for (int i = 0; i < 10; ++i) {
        resetState(state);
        auto start = high_resolution_clock::now();
        for (int i = 0; i < numSteps; ++i) {
            integrator.takeOneStep(step);
        }
        auto end = high_resolution_clock::now();
        double time_elapsed = duration_cast<microseconds>(end - start).count();
        time_elapsed /= 1.0e6;
        integration_times[i] = time_elapsed;
        real_time_factors[i] = time / time_elapsed;

        // Final energy
        // ------------
        const State& finalState = integrator.getAdvancedState();
        system.realize(finalState, SimTK::Stage::Dynamics);
        final_energies[i] = system.calcEnergy(finalState);;
    }
    j["integration_time"] = SimTK::mean(integration_times);
    j["real_time_factor"] = SimTK::mean(real_time_factors);
    double final_energy = SimTK::mean(final_energies);
    j["energy_conservation"] = final_energy - initial_energy;

    std::ofstream file(output_file, std::ios::out | std::ios::binary);
    if (file) {
        file << j.dump(4);
    }
}
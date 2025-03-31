#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Benchmark a simulation of an N-link pendulum in Simbody.

Usage:
  benchmark_simbody_pendulum_rk4 <nlinks> <output> [--time <time>] [--step <step>]
  benchmark_simbody_pendulum_rk4 -h | --help

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

        k1 = Vector(advancedState.getY().size(), 0.0);
        k2 = Vector(advancedState.getY().size(), 0.0);
        k3 = Vector(advancedState.getY().size(), 0.0);
        k4 = Vector(advancedState.getY().size(), 0.0);
    }

    const System& getSystem() const {
        return system;
    }

    void setAdvancedState(const Real& t, const Vector& y) {
        advancedState.updY() = y;
        advancedState.updTime() = t;
    }

    const State& getAdvancedState() const { return advancedState; }
    Real getAdvancedTime() const { return advancedState.getTime(); }

    const Real& getPreviousTime() const { return tPrev; }
    const Vector& getPreviousY() const { return yPrev; }
    const Vector& getPreviousYDot() const { return yDotPrev; }

    void realizeStateDerivatives(const SimTK::State& state) {
        if (state.getSystemStage() < Stage::Acceleration) {
            getSystem().realize(state, Stage::Acceleration);
        }       
    }

    void setAdvancedStateAndRealizeDerivatives(const Real& t, const Vector& y) {
        setAdvancedState(t, y);
        realizeStateDerivatives(getAdvancedState());
    }

    void setPreviousStateFromAdvancedState() {
        tPrev = advancedState.getTime();
        yPrev = advancedState.getY();
        yDotPrev = advancedState.getYDot();
    }

    void takeOneStep(Real h) {
        takeOneRungeKutta4Step(h);
        setPreviousStateFromAdvancedState();
    }

    // Standard Runge-Kutta 4th order method.
    //
    //  0  |
    // 1/2 | 1/2
    // 1/2 |  0  1/2
    //  1  |  0   0   1
    // --------------------
    //     | 1/6 2/6 2/6 1/6
    void takeOneRungeKutta4Step(Real h) {
        const Real t0 = getPreviousTime();
        const Vector& y0 = getPreviousY();
        const Vector& f0 = getPreviousYDot();
        const Real t1 = t0 + h;

        k1 = f0;

        setAdvancedStateAndRealizeDerivatives(t0 + 0.5*h, y0 + h*0.5*k1);
        k2 = getAdvancedState().getYDot();

        setAdvancedStateAndRealizeDerivatives(t0 + 0.5*h, y0 + h*0.5*k2);
        k3 = getAdvancedState().getYDot();

        setAdvancedStateAndRealizeDerivatives(t0 + h, y0 + h*k3);
        k4 = getAdvancedState().getYDot();

        setAdvancedStateAndRealizeDerivatives(t1, 
                y0 + h*(1.0/6.0)*(k1 + 2*k2 + 2*k3 + k4));
    }

private:
    const System& system;
    State advancedState;
    Real tPrev;
    Vector yPrev;
    Vector yDotPrev;

    Vector k1;
    Vector k2;
    Vector k3;
    Vector k4;
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
        OPENSIM_THROW_IF(step <= 0, OpenSim::Exception, 
                "Step size must be positive.");
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
    int numSteps = int(time / step);
    for (int i = 0; i < 10; ++i) {
        resetState(state);
        for (int i = 0; i < numSteps; ++i) {
            integrator.takeOneStep(step);
        }

        // Final energy
        // ------------
        const State& finalState = integrator.getAdvancedState();
        system.realize(finalState, SimTK::Stage::Dynamics);
        final_energies[i] = system.calcEnergy(finalState);;
    }
    double final_energy = SimTK::mean(final_energies);
    j["energy_conservation"] = final_energy - initial_energy;

    std::ofstream file(output_file, std::ios::out | std::ios::binary);
    if (file) {
        file << j.dump(4);
    }
}
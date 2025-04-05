#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <OpenSim/Moco/osimMoco.h>
#include "../../utilities/utilities.h"
#include <OpenSim/Simulation/VisualizerUtilities.h>

using namespace SimTK;

int main() {

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::microseconds;

    Logger::setLevel(Logger::Level::Error);

    // Disable logging.
    Model model("RajagopalFunctionBasedPathsDGFContact_nomuscles.osim");
    SimTK::State state = model.initSystem();

    // Helper function to reset the state.
    Vector defaultY = state.getY();
    auto resetState = [&](SimTK::State& state) {
        state.setTime(0);
        state.updY() = defaultY;
    };

    const SimTK::MultibodySystem& system = model.getMultibodySystem();
    system.realize(state, SimTK::Stage::Dynamics);
    std::cout << "initial energy: " << system.calcEnergy(state) << std::endl;
    
    SimTK::SemiExplicitEuler2Integrator integrator(system);
    integrator.setFixedStepSize(0.001);
    SimTK::TimeStepper timeStepper(system, integrator);
    timeStepper.initialize(state);
    auto start = high_resolution_clock::now();
    timeStepper.stepTo(1.0);
    auto end = high_resolution_clock::now();
    double time_elapsed = duration_cast<microseconds>(end - start).count();
    time_elapsed /= 1.0e6;

    state = timeStepper.getState();
    system.realize(state, SimTK::Stage::Dynamics);
    std::cout << "final energy: " << system.calcEnergy(state) << std::endl;
    std::cout << "TimeStepper time elapsed: " << time_elapsed << " seconds" << std::endl;


    resetState(state);
    Manager manager(model);
    // manager.setIntegratorMinimumStepSize(0.001);
    // manager.setIntegratorMaximumStepSize(0.001);
    manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKuttaMerson);
    manager.initialize(state);
    start = high_resolution_clock::now();
    manager.integrate(2.0);
    end = high_resolution_clock::now();
    time_elapsed = duration_cast<microseconds>(end - start).count();
    time_elapsed /= 1.0e6;
    std::cout << "Manager time elapsed: " << time_elapsed << " seconds" << std::endl;

    TimeSeriesTable table = manager.getStatesTable();
    VisualizerUtilities::showMotion(model, table);



    return 0;
} 
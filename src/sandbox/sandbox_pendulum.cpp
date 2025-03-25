#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <OpenSim/Moco/osimMoco.h>
#include "../../utilities/utilities.h"

using namespace SimTK;


State simulatePendulum(const MultibodySystem& system, State& state) {
    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    state.setTime(0);
    ts.initialize(state);
    ts.stepTo(5.0);
    State finalState = ts.getState();

    return finalState;
}

int main() {
    // The number of links in the pendulum.
    int numLinks = 5;

    // N-link pendulum defined directly in Simbody.
    PendulumSystem pendulumSystem(numLinks);
    const MultibodySystem& system = pendulumSystem.getMultibodySystem();
    State state = system.realizeTopology();
    SimTK::Vector q(state.getNQ(), 0.001);
    state.updQ() = q;
    State finalStateSimbody = simulatePendulum(system, state);

    // N-link pendulum defined in OpenSim.
    OpenSim::Model model = createOpenSimPendulum(numLinks);
    SimTK::State osimState = model.initSystem();
    osimState.updQ() = q;
    const MultibodySystem& osimSystem = model.getMultibodySystem();
    State finalStateOpenSim = simulatePendulum(osimSystem, osimState);

    SimTK_TEST_EQ(finalStateSimbody.getY(), finalStateOpenSim.getY());
    std::cout << "Pendulum simulation successful!" << std::endl;
    std::cout << "Final State Simbody Y: " << finalStateSimbody.getY() << std::endl;
    std::cout << "Final State OpenSim Y: " << finalStateOpenSim.getY() << std::endl;

    return 0;
}
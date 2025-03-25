#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <OpenSim/Moco/osimMoco.h>
#include "../../utilities/utilities.h"

using namespace SimTK;

int main() {

    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::Gravity gravity(forces, matter, -YAxis, 9.8);

    SimTK::Body::Rigid bodyInfo(MassProperties(1.0, Vec3(0), UnitInertia(1)));
    MobilizedBody::Pin pendulum0(matter.Ground(), Transform(Vec3(0)),
            bodyInfo, Transform(Vec3(-1, 0, 0)));
    for (int i = 1; i < 5; ++i) {
        pendulum0 = MobilizedBody::Pin(pendulum0, Transform(Vec3(0)),
                bodyInfo, Transform(Vec3(-1, 0, 0)));
    }

    // Set up visualization.
    system.setUseUniformBackground(true);
    Visualizer viz(system);
    system.addEventReporter(new Visualizer::Reporter(viz, 0.01));

    // Initialize the system and state.
    State state = system.realizeTopology();
    pendulum0.setRate(state, 5.0);

    // Initialize the system and state.
    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    state.setTime(0);
    ts.initialize(state);
    ts.stepTo(10.0);

    return 0;
} 
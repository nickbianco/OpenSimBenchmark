#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <OpenSim/Moco/osimMoco.h>
#include "../../utilities/utilities.h"

using namespace SimTK;

int main() {

    Model model("RajagopalFunctionBasedPathsDGF_noactdyn_nopassive.osim");
    State state = model.initSystem();
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    SimTK::TimeStepper timeStepper(model.getMultibodySystem(), integrator);
    timeStepper.initialize(state);
    timeStepper.stepTo(5.0);


    return 0;
} 
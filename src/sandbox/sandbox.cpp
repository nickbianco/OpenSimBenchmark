#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include "../../utilities/utilities.h"
#include <OpenSim/Simulation/VisualizerUtilities.h>

using namespace SimTK;
using namespace OpenSim;

int main() {

    Model model("Gait3D.osim");
    model.initSystem();

    // VisualizerUtilities::showModel(model);

    // Add a discrete controller to the model.
    DiscreteController* controller = new DiscreteController();
    controller->setName("controller");
    model.addController(controller);
    SimTK::State state = model.initSystem();

    // Randomize the initial speeds.
    state.updU() = 2.0*SimTK::Test::randVector(state.getNU());

    // Default controls
    // ----------------
    Vector controls(model.getNumControls(), 0.1);
    model.getComponent<DiscreteController>("/controllerset/controller").
        setDiscreteControls(state, controls);

    bool visualize = false;
    Manager manager(model);
    manager.setIntegratorMethod(Manager::IntegratorMethod::CPodes);
    manager.setIntegratorAccuracy(1e-2);
    manager.setPerformAnalyses(visualize);
    manager.setWriteToStorage(visualize);
    manager.initialize(state);
    manager.integrate(10.0);

    TimeSeriesTable statesTable = manager.getStatesTable();
    if (visualize) {
        VisualizerUtilities::showMotion(model, statesTable);
    }

    return 0;
}
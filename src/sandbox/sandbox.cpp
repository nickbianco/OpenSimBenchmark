#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include "../../utilities/utilities.h"
#include <OpenSim/Simulation/VisualizerUtilities.h>

using namespace SimTK;
using namespace OpenSim;

int main() {

    Model model("Gait3D.osim");
    model.initSystem();
    // model.setUseVisualizer(true);

    // VisualizerUtilities::showModel(model);

    // Add a discrete controller to the model.
    DiscreteController* controller = new DiscreteController();
    controller->setName("controller");
    model.addController(controller);
    SimTK::State state = model.initSystem();

    // Randomize the initial speeds.
    state.updU() = SimTK::Test::randVector(state.getNU());

    // Default controls
    // ----------------
    Vector controls(model.getNumControls(), 0.1);
    model.getComponent<DiscreteController>("/controllerset/controller").
        setDiscreteControls(state, controls);

    bool visualize = true;
    Manager manager(model);
    manager.setIntegratorMethod(Manager::IntegratorMethod::CPodes);
    manager.setIntegratorAccuracy(1e-3);
    manager.setPerformAnalyses(visualize);
    manager.setWriteToStorage(visualize);
    manager.setRecordStatesTrajectory(visualize);
    manager.initialize(state);
    manager.integrate(5.0);

    TimeSeriesTable statesTable = manager.getStatesTable();
    if (visualize) {
        VisualizerUtilities::showMotion(model, statesTable);
        // StatesDocument statesDoc = manager.getStatesTrajectory().exportToStatesDocument(model);
        // statesDoc.serialize("test.ostates");
    }

    return 0;
}
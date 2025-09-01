#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include "../../utilities/utilities.h"
#include <OpenSim/Simulation/VisualizerUtilities.h>

using namespace SimTK;
using namespace OpenSim;

int main() {

    Model model("Gait3DMillard.osim");
    model.initSystem();

    // Model model("Rajagopal22MusclesContact.osim");
    // model.initSystem();

    // Model model("RajagopalContact.osim");
    // model.initSystem();

    for (auto& path : model.updComponentList<Scholz2015GeometryPath>()) {
        path.setAlgorithm("MinimumLength");
    }

    // model.setUseVisualizer(true);

    // VisualizerUtilities::showModel(model);

    // Add a discrete controller to the model.
    DiscreteController* controller = new DiscreteController();
    controller->setName("controller");
    model.addController(controller);
    SimTK::State state = model.initSystem();



    // Vector controls(model.getNumControls(), 1.0);
    // controller->setDiscreteControls(state, controls);

    Manager manager(model);
    manager.setIntegratorMethod(Manager::IntegratorMethod::CPodes);
    // resetState(state);
    manager.initialize(state);
    manager.integrate(20.0);

    // TimeSeriesTable statesTable = manager.getStatesTable();
    // VisualizerUtilities::showMotion(model, statesTable);

    return 0;
}
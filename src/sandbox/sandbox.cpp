#include <OpenSim/OpenSim.h>
#include <OpenSim/Moco/osimMoco.h>
#include "../../utilities/utilities.h"

int main() {

    std::string models_path = "../../../models/";
    std::string model_name = "RajagopalFunctionBasedPaths";
    ModelProcessor modelProcessor = 
            ModelProcessor(models_path + model_name + ".osim") |
            ModOpReplaceMusclesWithDeGrooteFregly2016() |
            // ModOpIgnorePassiveFiberForcesDGF() |
            // ModOpPassiveFiberStrainAtOneNormForceDGF(0.6) |
            // ModOpIgnoreTendonCompliance() |
            // ModOpIgnoreActivationDynamics() |
            ModOpScaleActiveFiberForceCurveWidthDGF(1.0);

    Model model = modelProcessor.process();
    SimTK::State state = model.initSystem();

    state.setTime(0);
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    SimTK::TimeStepper timeStepper(model.getMultibodySystem(), integrator);
    timeStepper.initialize(state);

    auto start = std::chrono::high_resolution_clock::now();
    timeStepper.stepTo(1.0);
    auto end = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // Convert to milliseconds
    time /= 1e3;
    std::cout << "Time: " << time  << " milliseconds" << std::endl;

    std::cout << "num steps attempted: " << integrator.getNumStepsAttempted() << std::endl;
    std::cout << "num steps taken: " << integrator.getNumStepsTaken() << std::endl;
    std::cout << "num realizations: " << integrator.getNumRealizations() << std::endl;
    std::cout << "initial step size: " << integrator.getActualInitialStepSizeTaken() << std::endl;
    std::cout << "final step size: " << integrator.getPreviousStepSizeTaken() << std::endl;
    std::cout << "time per realization: " << time / integrator.getNumRealizations() << " milliseconds" << std::endl;


    return 0;
} 
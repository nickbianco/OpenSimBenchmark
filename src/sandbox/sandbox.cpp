#include <OpenSim/OpenSim.h>
#include "../utilities/utilities.h"

int main() {

    std::string models_path = "../../../models/RajagopalLaiUhlrich2023/";
    std::string model_name = "RajagopalLaiUhlrich2023_noconstraints_nomuscles";
    Model model(models_path + model_name + ".osim");
    // model.initSystem();
    // add_controller(model);
    SimTK::State state = model.initSystem();

    state.setTime(0);
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    SimTK::TimeStepper timeStepper(model.getMultibodySystem(), integrator);
    timeStepper.initialize(state);

    auto start = std::chrono::high_resolution_clock::now();
    timeStepper.stepTo(1.0);
    auto end = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "Time: " << time  << " microseconds" << std::endl;

    std::cout << "num steps attempted: " << integrator.getNumStepsAttempted() << std::endl;
    std::cout << "num steps taken: " << integrator.getNumStepsTaken() << std::endl;
    std::cout << "num realizations: " << integrator.getNumRealizations() << std::endl;
    std::cout << "initial step size: " << integrator.getActualInitialStepSizeTaken() << std::endl;
    std::cout << "final step size: " << integrator.getPreviousStepSizeTaken() << std::endl;
    std::cout << "time per realization: " << time / integrator.getNumRealizations() << " microseconds" << std::endl;


    return 0;
} 
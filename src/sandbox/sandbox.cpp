#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <OpenSim/Moco/osimMoco.h>
#include "../../utilities/utilities.h"
#include <OpenSim/Simulation/VisualizerUtilities.h>

using namespace SimTK;


class AccelerationReporter : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(AccelerationReporter, ModelComponent);

public:
    AccelerationReporter() : ModelComponent() {}

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        system.addEventReporter(new TextDataEventReporter
            (system, new MyEvaluateAcceleration(), 0.001));
    }

private:
    class MyEvaluateAcceleration : public TextDataEventReporter::UserFunction<Real> {
    public:
        Real evaluate(const System& system, const State& state) override {
            const MultibodySystem& mbs = MultibodySystem::downcast(system);
            mbs.realize(state, Stage::Acceleration);
            return state.getUDot().normRMS();
        }
    };
};
class SmoothSphereHalfSpaceReporter : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(SmoothSphereHalfSpaceReporter, ModelComponent);

public:
SmoothSphereHalfSpaceReporter() : ModelComponent() {}

protected:
    void extendConnectToModel(Model& model) override {
        for (const auto& force : model.getComponentList<OpenSim::SmoothSphereHalfSpaceForce>()) {
            m_forces.emplace_back(&force);
        }
    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        system.addEventReporter(new TextDataEventReporter
            (system, new MyEvaluateForces(m_forces), 0.001));
    }

private:
    class MyEvaluateForces : public TextDataEventReporter::UserFunction<Vector> {
    public:
        MyEvaluateForces(const std::vector<SimTK::ReferencePtr<const OpenSim::SmoothSphereHalfSpaceForce>>& forces)
            : m_forces(forces) {}

        Vector evaluate(const System& system, const State& state) override {
            const MultibodySystem& mbs = MultibodySystem::downcast(system);
            mbs.realize(state, Stage::Acceleration);
            SimTK::Vector forces(m_forces.size(), 0.0);
            for (size_t i = 0; i < m_forces.size(); ++i) {
                const auto& force = m_forces[i];
                const auto& contactForce = force->getSphereForce(state);
                forces[i] = contactForce.norm();
            }

            return forces;
        }
    private:
        const std::vector<SimTK::ReferencePtr<const OpenSim::SmoothSphereHalfSpaceForce>>& m_forces;

    };

    std::vector<SimTK::ReferencePtr<const OpenSim::SmoothSphereHalfSpaceForce>> m_forces;
};

class VisualizerReporter : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(VisualizerReporter, ModelComponent);

public:
    VisualizerReporter() : ModelComponent() {}

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        // system.setUseUniformBackground(true);
        Visualizer viz(system);
        system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
    }
};


void dumpIntegratorStats(double startCPU, double startTime, 
        const Integrator& integ) {
    std::cout << "DONE: Simulated " << integ.getTime() << " seconds in " <<
    realTime()-startTime << " elapsed s, CPU="<< cpuTime()-startCPU << "s\n";

    const int evals = integ.getNumRealizations();
    std::cout << "\nUsed "  << integ.getNumStepsTaken() << " steps, avg step=" 
    << (1000*integ.getTime())/integ.getNumStepsTaken() << "ms " 
    << (1000*integ.getTime())/evals << "ms/eval\n";

    printf("Used Integrator %s at accuracy %g:\n", 
    integ.getMethodName(), integ.getAccuracyInUse());
    printf("# STEPS/ATTEMPTS = %d/%d\n",  integ.getNumStepsTaken(), 
                        integ.getNumStepsAttempted());
    printf("# ERR TEST FAILS = %d\n",     integ.getNumErrorTestFailures());
    printf("# REALIZE/PROJECT = %d/%d\n", integ.getNumRealizations(), 
                        integ.getNumProjections());
}

int main() {

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::microseconds;


    // Disable logging.
    Logger::setLevel(Logger::Level::Error);

    // Load the model.
    // Model model("RajagopalFunctionBasedPathsDGFContact_noactdyn_nopassive.osim");
    Model model("RajagopalFunctionBasedPathsDGFContact_nomuscles.osim"); 
    model.initSystem();
    // model.addModelComponent(new AccelerationReporter());
    // model.addModelComponent(new VisualizerReporter()); 
    // model.addModelComponent(new SmoothSphereHalfSpaceReporter());
    SimTK::State state = model.initSystem();

    const MultibodySystem& system = model.getMultibodySystem();

    // Helper function to reset the state.
    Vector defaultY = state.getY();
    auto resetState = [&](SimTK::State& state) {
        state.setTime(0);
        state.updY() = defaultY;
        // state.updU() = 0.1*SimTK::Test::randVector(state.getNU());
    };
    
    // SimTK::RungeKuttaMersonIntegrator integ(system);
    SimTK::CPodesIntegrator integ(system,
        SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    // integ.setFixedStepSize(0.001);
    // integ.setMinimumStepSize(1e-6);
    SimTK::TimeStepper timeStepper(system, integ);

    resetState(state);
    timeStepper.initialize(state);

    const double startCPU = cpuTime(), startTime = realTime();
    timeStepper.stepTo(1.0);
    dumpIntegratorStats(startCPU, startTime, integ);




    return 0;
} 
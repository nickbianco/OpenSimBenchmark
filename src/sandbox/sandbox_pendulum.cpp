#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <OpenSim/Moco/osimMoco.h>
#include "../../utilities/utilities.h"

using namespace SimTK;


void resetState(SimTK::State& state) {
    // Reset the state to the initial state.
    state.setTime(0);
    state.updQ() = SimTK::Vector(state.getNQ(), 0.0);
    state.updQ()[0] = SimTK::Pi / 4.0;
    state.updU() = SimTK::Vector(state.getNU(), 0.0);
}


class MyIntegrator {

public:
    MyIntegrator(const System& system, const State& initialState) : system(system) {
        advancedState = system.realizeTopology();
        advancedState.setTime(initialState.getTime());
        advancedState.updY() = initialState.getY();
        setPreviousStateFromAdvancedState();

        k1 = Vector(advancedState.getY().size(), 0.0);
        k2 = Vector(advancedState.getY().size(), 0.0);
        k3 = Vector(advancedState.getY().size(), 0.0);
        k4 = Vector(advancedState.getY().size(), 0.0);
    }

    const System& getSystem() const {
        return system;
    }

    void setAdvancedState(const Real& t, const Vector& y) {
        advancedState.updY() = y;
        advancedState.updTime() = t;
    }

    const State& getAdvancedState() const { return advancedState; }
    Real getAdvancedTime() const { return advancedState.getTime(); }

    const Real& getPreviousTime() const { return tPrev; }
    const Vector& getPreviousY() const { return yPrev; }
    const Vector& getPreviousYDot() const { return yDotPrev; }

    void realizeStateDerivatives(const SimTK::State& state) {
        if (state.getSystemStage() < Stage::Acceleration) {
            getSystem().realize(state, Stage::Acceleration);
        }       
    }

    void setAdvancedStateAndRealizeDerivatives(const Real& t, const Vector& y) {
        setAdvancedState(t, y);
        realizeStateDerivatives(getAdvancedState());
    }

    void setPreviousStateFromAdvancedState() {
        tPrev = advancedState.getTime();
        yPrev = advancedState.getY();
        yDotPrev = advancedState.getYDot();
    }

    void takeOneStep(Real h) {
        takeOneRungeKutta4Step(h);
        setPreviousStateFromAdvancedState();
    }

    //  0  |
    // 1/2 | 1/2
    // 1/2 |  0  1/2
    //  1  |  0   0   1
    // --------------------
    //     | 1/6 2/6 2/6 1/6
    // 
    void takeOneRungeKutta4Step(Real h) {
        const Real t0 = getPreviousTime();
        const Vector& y0 = getPreviousY();
        const Vector& f0 = getPreviousYDot();
        const Real t1 = t0 + h;

        k1 = f0;

        setAdvancedStateAndRealizeDerivatives(t0 + 0.5*h, y0 + h*0.5*k1);
        k2 = getAdvancedState().getYDot();

        setAdvancedStateAndRealizeDerivatives(t0 + 0.5*h, y0 + h*0.5*k2);
        k3 = getAdvancedState().getYDot();

        setAdvancedStateAndRealizeDerivatives(t0 + h, y0 + h*k3);
        k4 = getAdvancedState().getYDot();

        setAdvancedStateAndRealizeDerivatives(t1, 
                y0 + h*(1.0/6.0)*(k1 + 2*k2 + 2*k3 + k4));
    }

private:
    const System& system;
    State advancedState;
    Real tPrev;
    Vector yPrev;
    Vector yDotPrev;

    Vector k1;
    Vector k2;
    Vector k3;
    Vector k4;
};


int main() {
    // The number of links in the pendulum.
    int numLinks = 100;

    // N-link pendulum defined directly in Simbody.
    PendulumSystem pendulumSystem(numLinks);
    const MultibodySystem& system = pendulumSystem.getMultibodySystem();
    State state = system.realizeTopology();
    resetState(state);

    system.realize(state, Stage::Dynamics);
    Real initialEnergy = system.calcEnergy(state);

    resetState(state);
    MyIntegrator integrator(system, state);
    Real h = 0.001;
    int steps = 1000;
    for (int i = 0; i < steps; ++i) {
        integrator.takeOneStep(h);
        const State& advancedState = integrator.getAdvancedState();
        const Real t = advancedState.getTime();
        const Vector& y = advancedState.getY();
        std::cout << "t: " << t << ", y: " << y << std::endl;
    }

    const State& finalState = integrator.getAdvancedState();
    system.realize(state, Stage::Dynamics);
    Real finalEnergy = system.calcEnergy(state);
    Real energyError = finalEnergy - initialEnergy;
    std::cout << "Initial energy: " << initialEnergy << std::endl;
    std::cout << "Final energy: " << finalEnergy << std::endl;
    std::cout << "Energy error: " << energyError << std::endl;

    return 0;
}
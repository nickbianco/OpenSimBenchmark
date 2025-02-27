#include <OpenSim/OpenSim.h>

using namespace OpenSim;

class StateGenerator {
public:
    StateGenerator(Model model) : model(model) {
        state = model.initSystem();

        auto coordinates = model.getCoordinatesInMultibodyTreeOrder();
        OPENSIM_THROW_IF(coordinates.size() != state.getNQ(), Exception, 
                "The number of coordinates does not match the number of "
                "generalized coordinates in the state.");

        q_bounds.reserve(coordinates.size());
        u_bounds.reserve(coordinates.size());
        for (int i = 0; i < coordinates.size(); ++i) {
            q_bounds.emplace_back(coordinates[i]->getRangeMin(), 
                                  coordinates[i]->getRangeMax());
            u_bounds.emplace_back(-10, 10);      
        }
    }

    SimTK::Vector getRandomQ() {
        SimTK::Vector q(state.getNQ());
        for (int i = 0; i < q.size(); ++i) {
            SimTK::Random::Uniform random(q_bounds[i].first, q_bounds[i].second);
            q[i] = random.getValue();
        }
        return q;
    }

    SimTK::Vector getRandomU() {
        SimTK::Vector u(state.getNU());
        for (int i = 0; i < u.size(); ++i) {
            SimTK::Random::Uniform random(u_bounds[i].first, u_bounds[i].second);
            u[i] = random.getValue();
        }
        return u;
    }

    SimTK::Vector getRandomZ() {
        SimTK::Vector z(state.getNZ());
        for (int i = 0; i < z.size(); ++i) {
            SimTK::Random::Uniform random(0, 1);
            z[i] = random.getValue();
        }
        return z;
    }

    SimTK::Vector getRandomY() {
        SimTK::Vector y(state.getNY());
        y.updBlock(0, 0, state.getNQ(), 1) = getRandomQ();
        y.updBlock(state.getNQ(), 0, state.getNU(), 1) = getRandomU();
        y.updBlock(state.getNQ() + state.getNU(), 0, state.getNZ(), 1) = 
                getRandomZ();
        return y;
    }

    SimTK::Vector getRandomControls() {
        SimTK::Vector controls = model.getDefaultControls();

        SimTK::Vector control(1, 0.0);
        for (const auto& actu : model.getComponentList<ScalarActuator>()) {
            SimTK::Real minControl = actu.getMinControl();
            SimTK::Real maxControl = actu.getMaxControl();
            SimTK::Random::Uniform random(minControl, maxControl);
            control[0] = random.getValue();
            actu.setControls(control, controls);
        }
        
        return controls;
    }

    void printBounds() {
        model.initSystem();
        auto coordinates = model.getCoordinatesInMultibodyTreeOrder();
        std::cout << std::endl;
        std::cout << "Coordinate bounds" << std::endl;
        std::cout << "-----------------" << std::endl;
        for (int i = 0; i < coordinates.size(); ++i) {
            std::cout << coordinates[i]->getName() 
                      << " - value: [" 
                      << q_bounds[i].first << ", " 
                      << q_bounds[i].second << "], speed: [" 
                      << u_bounds[i].first << ", " 
                      << u_bounds[i].second << "]" 
                      << std::endl;
        }
        std::cout << std::endl;
    }
   
private:
    Model model;
    SimTK::State state;
    std::vector<std::pair<double, double>> q_bounds;
    std::vector<std::pair<double, double>> u_bounds;
};
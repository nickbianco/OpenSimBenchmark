#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <docopt/docopt.h>

using namespace OpenSim;
using namespace SimTK;

class StateGenerator {
public:
    StateGenerator(Model model) : m_model(model) {
        m_state = m_model.initSystem();

        auto coordinates = m_model.getCoordinatesInMultibodyTreeOrder();
        OPENSIM_THROW_IF(coordinates.size() != m_state.getNQ(), 
                OpenSim::Exception, 
                "The number of coordinates does not match the number of "
                "generalized coordinates in the state.");

        q_bounds.reserve(coordinates.size());
        u_bounds.reserve(coordinates.size());
        for (int i = 0; i < coordinates.size(); ++i) {
            q_bounds.emplace_back(-0.1*coordinates[i]->getRangeMin(), 
                                  0.1*coordinates[i]->getRangeMax());
            if (coordinates[i]->getMotionType() == 
                    Coordinate::MotionType::Translational) {
                u_bounds.emplace_back(-0.1, 0.1);
            } else {    
                u_bounds.emplace_back(-1.0, 1.0);
            }

        }
    }

    SimTK::Vector getRandomQ() const {
        SimTK::Vector q(m_state.getNQ());
        for (int i = 0; i < q.size(); ++i) {
            SimTK::Random::Uniform random(q_bounds[i].first, q_bounds[i].second);
            q[i] = random.getValue();
        }
        return q;
    }

    SimTK::Vector getRandomU() const {
        SimTK::Vector u(m_state.getNU());
        for (int i = 0; i < u.size(); ++i) {
            SimTK::Random::Uniform random(u_bounds[i].first, u_bounds[i].second);
            u[i] = random.getValue();
        }
        return u;
    }

    SimTK::Vector getRandomZ() const {
        SimTK::Vector z(m_state.getNZ());
        for (int i = 0; i < z.size(); ++i) {
            // Most muscle states should behave well in this range. We mostly
            // do not want to generate states that are too close to zero or one.
            SimTK::Random::Uniform random(0.2, 0.3);
            z[i] = random.getValue();
        }
        return z;
    }

    SimTK::Vector getRandomY() const {
        SimTK::Vector y(m_state.getNY());
        y.updBlock(0, 0, m_state.getNQ(), 1) = getRandomQ();
        y.updBlock(m_state.getNQ(), 0, m_state.getNU(), 1) = getRandomU();
        y.updBlock(m_state.getNQ() + m_state.getNU(), 0, m_state.getNZ(), 1) = 
                getRandomZ();
        return y;
    }

    SimTK::Vector getRandomControls() const {
        SimTK::Vector controls = m_model.getDefaultControls();

        SimTK::Vector control(1, 0.0);
        for (const auto& actu : m_model.getComponentList<ScalarActuator>()) {
            SimTK::Real minControl = actu.getMinControl();
            SimTK::Real maxControl = actu.getMaxControl();
            SimTK::Real range = maxControl - minControl;
            SimTK::Real buffer = 0.2 * range;
            minControl += buffer;
            maxControl -= buffer;
            SimTK::Random::Uniform random(minControl, maxControl);
            control[0] = random.getValue();
            actu.setControls(control, controls);
        }
        
        return controls;
    }

    void printBounds() const {
        auto coordinates = m_model.getCoordinatesInMultibodyTreeOrder();
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
    Model m_model;
    SimTK::State m_state;
    std::vector<std::pair<double, double>> q_bounds;
    std::vector<std::pair<double, double>> u_bounds;
};


// This implementation was copied from docopt.cpp; the only difference is that
// this function can throw exceptions (whereas `docopt::docopt()` is
// `noexcept`).
inline std::map<std::string, docopt::value>
parse_arguments(std::string const& doc,
            std::vector<std::string> const& argv,
            bool help = true,
            std::string const& version = {},
            bool options_first = false)
{
    using namespace docopt;
    try {
        return docopt_parse(doc, argv, help, !version.empty(), options_first);
    } catch (DocoptExitHelp const&) {
        log_cout(doc);
        std::exit(0);
    } catch (DocoptExitVersion const&) {
        log_cout(version);
        std::exit(0);
    } catch (DocoptLanguageError const& error) {
        log_error("Docopt usage string could not be parsed.");
        log_error(error.what());
        std::exit(-1);
    } catch (DocoptArgumentError const& error) {
        log_error(error.what());
        log_error("Use --help to get more information.");
        std::exit(-1);
    }
}

struct PendulumSystem {
    PendulumSystem(int numLinks) : m_system(), m_matter(m_system), 
                m_forces(m_system), m_gravity(m_forces, m_matter, -YAxis, 9.81) {        
        SimTK::Body::Rigid bodyInfo(MassProperties(1.0, Vec3(0), UnitInertia(1)));
        MobilizedBody::Pin pendulum0(m_matter.Ground(), Transform(Vec3(0)),
                bodyInfo, Transform(Vec3(0, 1, 0)));
        for (int i = 1; i < numLinks; ++i) {
            pendulum0 = MobilizedBody::Pin(pendulum0, Transform(Vec3(0)),
                    bodyInfo, Transform(Vec3(0, 1, 0)));
        }
        m_system.realizeTopology();
    }

    const MultibodySystem& getMultibodySystem() const { return m_system; }

    MultibodySystem                 m_system;
    GeneralForceSubsystem           m_forces;
    SimbodyMatterSubsystem          m_matter;
    SimTK::Force::Gravity           m_gravity;
};


inline Model createOpenSimPendulum(int numLinks) {
    OpenSim::Model model;
    model.setName(std::to_string(numLinks) + "_link_pendulum");
    model.setGravity(Vec3(0, -9.81, 0));
    const auto& ground = model.getGround();

    const OpenSim::PhysicalFrame* prevBody = &ground;
    for (int i = 0; i < numLinks; ++i) {
        const std::string istr = std::to_string(i);
        auto* bi = new OpenSim::Body("b" + istr, 1, Vec3(0), Inertia(1));
        model.addBody(bi);

        // Assume each body is 1 m long.
        auto* ji = new OpenSim::PinJoint("j" + istr, *prevBody, Vec3(0), 
                Vec3(0), *bi, Vec3(0, 1, 0), Vec3(0));
        auto& qi = ji->updCoordinate();
        qi.setName("q" + istr);
        model.addJoint(ji);

        prevBody = bi;
    }

    model.finalizeConnections();
    return model;
}
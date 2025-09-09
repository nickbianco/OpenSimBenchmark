#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include "docopt.h"

using namespace OpenSim;
using namespace SimTK;

// inline SimTK::Vector run_benchmarks(Model model, double time, double step,
//         double accuracy, Manager::IntegratorMethod integratorMethod) {

//     using std::chrono::high_resolution_clock;
//     using std::chrono::duration_cast;
//     using std::chrono::duration;
//     using std::chrono::microseconds;

//     // Initialize model, state, and results vector.
//     model.initSystem();
//     SimTK::Vector results(7, SimTK::NaN);

//     // Add a discrete controller to the model.
//     DiscreteController* controller = new DiscreteController();
//     controller->setName("controller");
//     model.addController(controller);
//     SimTK::State state = model.initSystem();

//     // Helper function to reset the state.
//     Vector defaultY = state.getY();
//     auto resetState = [&](SimTK::State& state) {
//         state.setTime(0);
//         state.updY() = defaultY;
//         state.updU() = 2.0*SimTK::Test::randVector(state.getNU());
//         // SimTK::Random::Uniform rand(0.1, 0.2);
//         Vector controls(model.getNumControls(), 0.1);
//         controller->setDiscreteControls(state, controls);
//     };

//     // Realize acceleration.
//     // ---------------------
//     SimTK::Vector acceleration_times(100, 0.0);
//     for (int i = 0; i < 100; ++i) {
//         resetState(state);
//         auto start = high_resolution_clock::now();
//         model.realizeAcceleration(state);
//         auto end = high_resolution_clock::now();
//         double time_elapsed = duration_cast<microseconds>(end - start).count();
//         time_elapsed /= 1.0e6;
//         acceleration_times[i] = time_elapsed;
//     }
//     results[0] = SimTK::mean(acceleration_times);

//     // One simulation step.
//     // --------------------
//     if (step > 0) {
//         Manager manager(model);
//         manager.setIntegratorMethod(integratorMethod);
//         manager.setIntegratorFixedStepSize(step);
//         SimTK::Vector step_times(100, 0.0);
//         for (int i = 0; i < 100; ++i) {
//             resetState(state);
//             manager.initialize(state);
//             auto start = high_resolution_clock::now();
//             manager.integrate(step);
//             auto end = high_resolution_clock::now();
//             double time_elapsed = duration_cast<microseconds>(end - start).count();
//             time_elapsed /= 1.0e6;
//             step_times[i] = time_elapsed;
//         }
//         results[1] = SimTK::mean(step_times);
//     }

//     // Initial energy
//     // --------------
//     resetState(state);
//     model.realizeDynamics(state);
//     double initial_energy = model.calcPotentialEnergy(state) +
//             model.calcKineticEnergy(state);

//     // Forward integration.
//     // --------------------
//     SimTK::Vector integration_times(10, 0.0);
//     SimTK::Vector real_time_factors(10, 0.0);
//     SimTK::Vector final_energies(10, 0.0);
//     SimTK::Vector num_steps(10, 0.0);
//     SimTK::Vector times_per_step(10, 0.0);
//     Manager manager(model);
//     manager.setIntegratorMethod(integratorMethod);
//     if (step > 0) {
//         manager.setIntegratorFixedStepSize(step);
//     } else {
//         manager.setIntegratorAccuracy(accuracy);
//     }
//     for (int i = 0; i < 10; ++i) {
//         resetState(state);
//         manager.initialize(state);
//         auto start = high_resolution_clock::now();
//         manager.integrate(time);
//         auto end = high_resolution_clock::now();
//         double time_elapsed = duration_cast<microseconds>(end - start).count();
//         time_elapsed /= 1.0e6;
//         integration_times[i] = time_elapsed;
//         real_time_factors[i] = time / time_elapsed;
//         num_steps[i] = manager.getIntegrator().getNumStepsTaken();
//         times_per_step[i] = time_elapsed / num_steps[i];

//         // Final energy
//         // ------------
//         const auto& finalState = manager.getState();
//         model.realizeDynamics(finalState);
//         final_energies[i] = model.calcPotentialEnergy(finalState) +
//                 model.calcKineticEnergy(finalState);
//     }
//     results[2] = SimTK::mean(integration_times);
//     results[3] = SimTK::mean(real_time_factors);
//     double final_energy = SimTK::mean(final_energies);
//     results[4] = final_energy - initial_energy;
//     results[5] = SimTK::mean(num_steps);
//     results[6] = SimTK::mean(times_per_step);

//     return results;
// }

inline SimTK::Vector run_benchmarks(Model model, double time, double step,
        double accuracy, Manager::IntegratorMethod integratorMethod) {

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::microseconds;

    // Initialize model, state, and results vector.
    model.initSystem();
    SimTK::Vector results(7, SimTK::NaN);

    // Add a discrete controller to the model.
    DiscreteController* controller = new DiscreteController();
    controller->setName("controller");
    model.addController(controller);
    SimTK::State state = model.initSystem();

    // Randomize the initial speeds.
    state.updU() = 2.0*SimTK::Test::randVector(state.getNU());

    // Default controls
    Vector controls(model.getNumControls(), 0.1);
    model.getComponent<DiscreteController>("/controllerset/controller").
        setDiscreteControls(state, controls);

    Manager manager(model);
    manager.setIntegratorMethod(Manager::IntegratorMethod::CPodes);
    manager.setIntegratorAccuracy(accuracy);
    manager.setPerformAnalyses(false);
    manager.setWriteToStorage(false);
    manager.initialize(state);
    auto start = high_resolution_clock::now();
    manager.integrate(time);
    auto end = high_resolution_clock::now();
    double time_elapsed = duration_cast<microseconds>(end - start).count();
    time_elapsed /= 1.0e6;
    results[2] = time_elapsed;
    results[3] = time / time_elapsed;

    return results;
}

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

class PointPathMuscle : public SimTK::Force::Custom::Implementation {
public:
    PointPathMuscle(const SimbodyMatterSubsystem& matter,
                    const Real& maxIsometricForce,
                    const Real& optimalFiberLength,
                    const Real& tendonSlackLength,
                    const Real& pennationAngle);

    void addPoint(MobilizedBodyIndex body, const Vec3& station);

    virtual void calcForce(const State& state,
                            Vector_<SpatialVec>& bodyForces,
                            Vector_<Vec3>& particleForces,
                            Vector& mobilityForces) const override;

    virtual Real calcPotentialEnergy(const State& state) const override;

    void addDecorativeLines(DecorationSubsystem& viz,
                            const DecorativeLine& line) const {
        for (int i=1; i < m_bodies.size(); ++i) {
            viz.addRubberBandLine(m_bodies[i-1], m_stations[i-1],
                m_bodies[i], m_stations[i], line);
        }
    }

    const MobilizedBody& getMobilizedBody(MobilizedBodyIndex body) const {
        return m_matter.getMobilizedBody(body);
    }

    Real calcTendonForce(const Real& tendonLength) const;
    Real calcActiveForceLengthMultiplier(const Real& normalizedFiberLength) const;
    Real calcActiveForceVelocityMultiplier(const Real& normalizedFiberVelocity) const;
    Real calcPassiveForceLengthMultiplier(const Real& normalizedFiberLength) const;
    Real calcFiberForce(const Real& activation,
                        const Real& normalizedFiberLength,
                        const Real& normalizedFiberVelocity) const;

    void realizeTopology(State& state) const override {
        m_actIx = m_matter.allocateZ(state, Vector(1, Real(0)));
    }

    void realizeAcceleration(const State& state) const override {
        const Real excitation = 0.1;
        const Real activation = getActivation(state);

        m_matter.updZDot(state)[m_actIx] =
            (excitation - activation) * (c1*excitation + c2);
    }

    Real getActivation(const State& state) const {
        return m_matter.getZ(state)[m_actIx];
    }

private:
    Array_<MobilizedBodyIndex> m_bodies;
    Array_<Vec3> m_stations;

    const SimbodyMatterSubsystem& m_matter;

    Real m_maxIsometricForce;
    Real m_optimalFiberLength;
    Real m_tendonSlackLength;
    Real m_cosPennationAngle;
    Real m_vmax = 10.0;
    Real m_fiberDamping = 0.1;

    // Tendon force-length curve.
    constexpr static Real cT1 = 260.972;
    constexpr static Real cT2 = 7.9706;
    // Fiber active force-length curve.
    constexpr static Real cL1 = 1.5;
    constexpr static Real cL2 = -2.75;
    constexpr static Real r1 = 0.46899;
    constexpr static Real r2 = 1.80528;
    // Fiber active force-velocity curve.
    constexpr static Real cV1 = 0.227;
    constexpr static Real cV2 = 0.5;
    constexpr static Real Fvmax = 1.6;
    // Fiber passive force-length curve.
    constexpr static Real cP1 = 1.08027;
    constexpr static Real cP2 = 1.27368;
    // Activation dynamics.
    constexpr static Real c1 = 75;
    constexpr static Real c2 = 25;

    // Topology cache; written only once by realizeTopology().
    mutable ZIndex m_actIx;
};


class Gait3D {
public:
    enum BodyType    {LeftFoot=0, RightFoot,
                      LeftShank, RightShank,
                      LeftThigh, RightThigh,
                      Pelvis, Torso};
    enum Muscle      {GlutMed_R, AddMag_R, Hamstrings_R, Bifemsh_R,
                      GlutMax_R, Iliopsoas_R, RectFem_R, Vasti_R,
                      Gastroc_R, Soleus_R, TibAnt_R, GlutMed_L,
                      AddMag_L, Hamstrings_L, Bifemsh_L,
                      GlutMax_L, Iliopsoas_L, RectFem_L, Vasti_L,
                      Gastroc_L, Soleus_L, TibAnt_L};
    enum ContactType {HuntCrossleyForce,
                      CompliantContactSubsystem,
                      ExponentialSpringForce};

    static const int NBodyType = Torso-LeftFoot+1;
    static const int NContacts = 3;

    Gait3D(ContactType contactType);

    void loadDefaultState(State& state);

    MultibodySystem                    m_system;
    SimbodyMatterSubsystem             m_matter;
    GeneralForceSubsystem              m_forces;
    ContactTrackerSubsystem            m_tracker;
    SimTK::CompliantContactSubsystem   m_compliantContacts;
    GeneralContactSubsystem            m_generalContacts;
    DecorationSubsystem                m_viz;
    SimTK::Force::Gravity              m_gravity;

    Vector m_mass;             // index by BodyType
    Vector_<Vec3> m_inertia;  // index by BodyType

    std::map<BodyType, SimTK::Body>     m_body;
    std::map<BodyType, MobilizedBody>   m_mobod;
    std::map<Muscle, PointPathMuscle*>  m_muscles;

private:
    static Real massData[NBodyType];
    static Vec3 inertiaData[NBodyType];
    static Vec3 leftContactPoints[NContacts], rightContactPoints[NContacts];

    // Contact parameters.
    constexpr static Real stiffness = 5e6;
    constexpr static Real dissipation = 1.0;
    constexpr static Real mu_static = 0.9;
    constexpr static Real mu_dynamic = 0.6;
    constexpr static Real mu_viscous = 0.;
    constexpr static Real transitionVelocity = .1;
    constexpr static Real radius = 0.02;
};

Real Gait3D::massData[] = {1.25, 1.25, 3.7075, 3.7075, 9.3014, 9.3014,
                           11.777, 34.2366};

Vec3 Gait3D::inertiaData[] = {Vec3(0.0014, 0.0039, 0.0041),
                              Vec3(0.0014, 0.0039, 0.0041),
                              Vec3(0.0504, 0.0051, 0.0511),
                              Vec3(0.0504, 0.0051, 0.0511),
                              Vec3(0.1339, 0.0351, 0.1412),
                              Vec3(0.1339, 0.0351, 0.1412),
                              Vec3(0.1028, 0.0871, 0.0579),
                              Vec3(1.4745, 0.7555, 1.4314)};

Vec3 Gait3D::leftContactPoints[] = {Vec3(-0.085, -0.015, 0.005), // heel
                                    Vec3(0.0425, -0.03, -0.041), // lateral toe
                                    Vec3(0.085, -0.03, 0.0275)}; // medial toe

Vec3 Gait3D::rightContactPoints[] = {Vec3(-0.085, -0.015, -0.005), // heel
                                     Vec3(0.0425, -0.03, 0.041),   // lateral toe
                                     Vec3(0.085, -0.03, -0.0275)}; // medial toe

Gait3D::Gait3D(ContactType contactType)
:   m_matter(m_system), m_forces(m_system), m_tracker(m_system),
    m_compliantContacts(m_system, m_tracker), m_generalContacts(m_system),
    m_viz(m_system), m_gravity(m_forces, m_matter, -YAxis, 9.81),
    m_mass(NBodyType, massData), m_inertia(NBodyType, inertiaData)
{
    // Create bodies
    m_body[Pelvis] = SimTK::Body::Rigid(MassProperties(m_mass[Pelvis], Vec3(0),
        Inertia(m_inertia[Pelvis])));
    m_body[Torso] = SimTK::Body::Rigid(MassProperties(m_mass[Torso], Vec3(0),
        Inertia(m_inertia[Torso])));
    m_body[LeftThigh] = SimTK::Body::Rigid(MassProperties(m_mass[LeftThigh], Vec3(0),
        Inertia(m_inertia[LeftThigh])));
    m_body[LeftShank] = SimTK::Body::Rigid(MassProperties(m_mass[LeftShank], Vec3(0),
        Inertia(m_inertia[LeftShank])));
    m_body[LeftFoot] = SimTK::Body::Rigid(MassProperties(m_mass[LeftFoot],  Vec3(0),
        Inertia(m_inertia[LeftFoot])));
    m_body[RightThigh] = SimTK::Body::Rigid(MassProperties(m_mass[RightThigh], Vec3(0),
        Inertia(m_inertia[RightThigh])));
    m_body[RightShank] = SimTK::Body::Rigid(MassProperties(m_mass[RightShank], Vec3(0),
        Inertia(m_inertia[RightShank])));
    m_body[RightFoot] = SimTK::Body::Rigid(MassProperties(m_mass[RightFoot],  Vec3(0),
        Inertia(m_inertia[RightFoot])));

    // Add body decorative geometry.
    Real opacity = 0.6;
    m_body[Pelvis].addDecoration(Vec3(-0.01, -0.05, 0),
            DecorativeEllipsoid(Vec3(0.07, 0.07, 0.12))
                    .setColor(Gray).setOpacity(opacity));

    m_body[Torso].addDecoration(Vec3(0),
            DecorativeEllipsoid(Vec3(0.1, 0.27, 0.1))
                    .setColor(Gray).setOpacity(opacity));
    m_body[Torso].addDecoration(Vec3(0, 0.38, 0),
            DecorativeSphere(0.09).setColor(Gray).setOpacity(opacity));

    m_body[LeftThigh].addDecoration(Vec3(0),
            DecorativeEllipsoid(Vec3(0.04, 0.2, 0.04))
                    .setColor(Gray).setOpacity(opacity));
    m_body[LeftShank].addDecoration(Vec3(0),
            DecorativeCylinder(0.02, 0.22)
                    .setColor(Gray).setOpacity(opacity));
    m_body[LeftFoot].addDecoration(Vec3(0),
            DecorativeEllipsoid(Vec3(0.1, 0.03, 0.05))
                    .setColor(Gray).setOpacity(opacity));

    m_body[RightThigh].addDecoration(Vec3(0),
            DecorativeEllipsoid(Vec3(0.04, 0.2, 0.04))
                    .setColor(Gray).setOpacity(opacity));
    m_body[RightShank].addDecoration(Vec3(0),
            DecorativeCylinder(0.02, 0.22)
                    .setColor(Gray).setOpacity(opacity));
    m_body[RightFoot].addDecoration(Vec3(0),
            DecorativeEllipsoid(Vec3(0.1, 0.03, 0.05))
                    .setColor(Gray).setOpacity(opacity));

    // Add contact decorative geometry.
    m_body[LeftFoot].addDecoration(leftContactPoints[0],
            DecorativeSphere(radius).setColor(Green));
    m_body[LeftFoot].addDecoration(leftContactPoints[1],
            DecorativeSphere(radius).setColor(Green));
    m_body[LeftFoot].addDecoration(leftContactPoints[2],
            DecorativeSphere(radius).setColor(Green));

    m_body[RightFoot].addDecoration(rightContactPoints[0],
            DecorativeSphere(radius).setColor(Green));
    m_body[RightFoot].addDecoration(rightContactPoints[1],
            DecorativeSphere(radius).setColor(Green));
    m_body[RightFoot].addDecoration(rightContactPoints[2],
            DecorativeSphere(radius).setColor(Green));

    if (contactType == ContactType::CompliantContactSubsystem) {
        m_compliantContacts.setTransitionVelocity(transitionVelocity);

        // Add ContactSurfaces to the feet.
        ContactCliqueId clique1 = ContactSurface::createNewContactClique();
        ContactMaterial material(stiffness, dissipation, mu_static,
                                mu_dynamic, mu_viscous);

        // Left foot
        // ---------
        // Heel sphere
        m_body[LeftFoot].addContactSurface(leftContactPoints[0],
            ContactSurface(SimTK::ContactGeometry::Sphere(radius), material)
            .joinClique(clique1));

        // Lateral toe sphere
        m_body[LeftFoot].addContactSurface(leftContactPoints[1],
            ContactSurface(SimTK::ContactGeometry::Sphere(radius), material)
            .joinClique(clique1));

        // Medial toe sphere
        m_body[LeftFoot].addContactSurface(leftContactPoints[2],
            ContactSurface(SimTK::ContactGeometry::Sphere(radius), material)
            .joinClique(clique1));

        // Right foot
        // ----------
        // Heel sphere
        m_body[RightFoot].addContactSurface(rightContactPoints[0],
            ContactSurface(SimTK::ContactGeometry::Sphere(radius), material)
            .joinClique(clique1));

        // Lateral toe sphere
        m_body[RightFoot].addContactSurface(rightContactPoints[1],
            ContactSurface(SimTK::ContactGeometry::Sphere(radius), material)
            .joinClique(clique1));

        // Medial toe sphere
        m_body[RightFoot].addContactSurface(rightContactPoints[2],
            ContactSurface(SimTK::ContactGeometry::Sphere(radius), material)
            .joinClique(clique1));

        // Half space
        // ----------
        // Half space normal is -x; must rotate to make it +y.
        m_matter.Ground().updBody().addContactSurface(Rotation(-Pi/2,ZAxis),
        ContactSurface(SimTK::ContactGeometry::HalfSpace(), material));
    }

    // Mobilized bodies
    // ----------------
    // Add pelvis.
    m_mobod[Pelvis] = MobilizedBody::Free(m_matter.Ground(), m_body[Pelvis]);

    // Add torso.
    m_mobod[Torso] = MobilizedBody::Ball(
            m_mobod[Pelvis], Vec3(0, 0.05, 0),
            m_body[Torso], Vec3(0, -0.25, 0));

    // Add left leg.
    m_mobod[LeftThigh] = MobilizedBody::Ball(
            m_mobod[Pelvis], Vec3(0, -0.0661, -0.0835),
            m_body[LeftThigh], Transform(Vec3(0, 0.17, 0)));
    m_mobod[LeftShank] = MobilizedBody::Pin(
            m_mobod[LeftThigh], Vec3(0, -0.226, 0),
            m_body[LeftShank], Transform(Vec3(0, 0.1867, 0)));
    m_mobod[LeftFoot] = MobilizedBody::Pin(
            m_mobod[LeftShank], Transform(Vec3(0, -0.2433, 0)),
            m_body[LeftFoot], Transform(Vec3(-0.05123, 0.01195, 0.00792)));

    // Add right leg.
    m_mobod[RightThigh] = MobilizedBody::Ball(
            m_mobod[Pelvis], Vec3(0, -0.0661, 0.0835),
            m_body[RightThigh], Transform(Vec3(0, 0.17, 0)));
    m_mobod[RightShank] = MobilizedBody::Pin(
            m_mobod[RightThigh], Vec3(0, -0.226, 0),
            m_body[RightShank], Transform(Vec3(0, 0.1867, 0)));
    m_mobod[RightFoot] = MobilizedBody::Pin(
            m_mobod[RightShank], Transform(Vec3(0, -0.2433, 0)),
            m_body[RightFoot], Transform(Vec3(-0.05123, 0.01195, -0.00792)));

    // We need the mobilized bodies to add exponential spring forces.
    if (contactType == ContactType::ExponentialSpringForce) {
        Transform groundFrame(Rotation(-0.5*Pi, XAxis), Vec3(0));

        ExponentialSpringParameters params;
        params.setNormalViscosity(dissipation);
        // params.setFrictionViscosity(dissipation);
        params.setInitialMuStatic(mu_static);
        params.setInitialMuKinetic(mu_dynamic);
        params.setSettleVelocity(transitionVelocity);

        SimTK::ExponentialSpringForce leftHeel(m_forces, groundFrame,
                m_mobod[LeftFoot], leftContactPoints[0], params);
        SimTK::ExponentialSpringForce leftLateralToe(m_forces, groundFrame,
                m_mobod[LeftFoot], leftContactPoints[1], params);
        SimTK::ExponentialSpringForce leftMedialToe(m_forces, groundFrame,
                m_mobod[LeftFoot], leftContactPoints[2], params);

        SimTK::ExponentialSpringForce rightHeel(m_forces, groundFrame,
                m_mobod[RightFoot], rightContactPoints[0], params);
        SimTK::ExponentialSpringForce rightLateralToe(m_forces, groundFrame,
                m_mobod[RightFoot], rightContactPoints[1], params);
        SimTK::ExponentialSpringForce rightMedialToe(m_forces, groundFrame,
                m_mobod[RightFoot], rightContactPoints[2], params);

    } else if (contactType == ContactType::HuntCrossleyForce) {

        ContactSetIndex setIndex = m_generalContacts.createContactSet();
        m_generalContacts.addBody(setIndex, m_matter.updGround(),
                SimTK::ContactGeometry::HalfSpace(),
                Transform(Rotation(-0.5*Pi, ZAxis), Vec3(0))); // y < 0

        m_generalContacts.addBody(setIndex, m_mobod[LeftFoot],
                SimTK::ContactGeometry::Sphere(radius),
                Transform(leftContactPoints[0]));
        m_generalContacts.addBody(setIndex, m_mobod[LeftFoot],
                SimTK::ContactGeometry::Sphere(radius),
                Transform(leftContactPoints[1]));
        m_generalContacts.addBody(setIndex, m_mobod[LeftFoot],
                SimTK::ContactGeometry::Sphere(radius),
                Transform(leftContactPoints[2]));

        m_generalContacts.addBody(setIndex, m_mobod[RightFoot],
                SimTK::ContactGeometry::Sphere(radius),
                Transform(rightContactPoints[0]));
        m_generalContacts.addBody(setIndex, m_mobod[RightFoot],
                SimTK::ContactGeometry::Sphere(radius),
                Transform(rightContactPoints[1]));
        m_generalContacts.addBody(setIndex, m_mobod[RightFoot],
                SimTK::ContactGeometry::Sphere(radius),
                Transform(rightContactPoints[2]));

        SimTK::HuntCrossleyForce hc(m_forces, m_generalContacts, setIndex);
        hc.setBodyParameters(ContactSurfaceIndex(0), stiffness, dissipation,
                mu_static, mu_dynamic, mu_viscous);
        hc.setBodyParameters(ContactSurfaceIndex(1), stiffness, dissipation,
                mu_static, mu_dynamic, mu_viscous);
        hc.setBodyParameters(ContactSurfaceIndex(2), stiffness, dissipation,
                mu_static, mu_dynamic, mu_viscous);
        hc.setBodyParameters(ContactSurfaceIndex(3), stiffness, dissipation,
                mu_static, mu_dynamic, mu_viscous);
        hc.setBodyParameters(ContactSurfaceIndex(4), stiffness, dissipation,
                mu_static, mu_dynamic, mu_viscous);
        hc.setBodyParameters(ContactSurfaceIndex(5), stiffness, dissipation,
                mu_static, mu_dynamic, mu_viscous);
        hc.setBodyParameters(ContactSurfaceIndex(6), stiffness, dissipation,
                mu_static, mu_dynamic, mu_viscous);
        hc.setTransitionVelocity(transitionVelocity);
    }

    // Joint damping forces.
    // -------------------
    Real jointDamping = 0.1;
    SimTK::Force::MobilityLinearDamper lumbarDamperX(m_forces, m_mobod[Torso],
            MobilizerUIndex(0), 10.0*jointDamping);
    SimTK::Force::MobilityLinearDamper lumbarDamperY(m_forces, m_mobod[Torso],
            MobilizerUIndex(1), 10.0*jointDamping);
    SimTK::Force::MobilityLinearDamper lumbarDamperZ(m_forces, m_mobod[Torso],
            MobilizerUIndex(2), 10.0*jointDamping);

    SimTK::Force::MobilityLinearDamper leftHipDamperX(m_forces, m_mobod[LeftThigh],
            MobilizerUIndex(0), jointDamping);
    SimTK::Force::MobilityLinearDamper leftHipDamperY(m_forces, m_mobod[LeftThigh],
            MobilizerUIndex(1), jointDamping);
    SimTK::Force::MobilityLinearDamper leftHipDamperZ(m_forces, m_mobod[LeftThigh],
            MobilizerUIndex(2), jointDamping);

    SimTK::Force::MobilityLinearDamper righHipDamperX(m_forces, m_mobod[RightThigh],
            MobilizerUIndex(0), jointDamping);
    SimTK::Force::MobilityLinearDamper rightHipDamperY(m_forces, m_mobod[RightThigh],
            MobilizerUIndex(1), jointDamping);
    SimTK::Force::MobilityLinearDamper rightHipDamperZ(m_forces, m_mobod[RightThigh],
            MobilizerUIndex(2), jointDamping);

    SimTK::Force::MobilityLinearDamper leftKneeDamper(m_forces, m_mobod[LeftShank],
            MobilizerUIndex(0), jointDamping);
    SimTK::Force::MobilityLinearDamper leftAnkleDamper(m_forces, m_mobod[LeftFoot],
            MobilizerUIndex(0), jointDamping);

    SimTK::Force::MobilityLinearDamper rightKneeDamper(m_forces, m_mobod[RightShank],
            MobilizerUIndex(0), jointDamping);
    SimTK::Force::MobilityLinearDamper rightAnkleDamper(m_forces, m_mobod[RightFoot],
            MobilizerUIndex(0), jointDamping);

    // Joint limit forces.
    // -------------------
    // SimTK::Force::MobilityLinearStop lumbarForceX(m_forces, m_mobod[Torso],
    //         MobilizerQIndex(1), 500, 9.02585,
    //         convertDegreesToRadians(-20.0), convertDegreesToRadians(20.0));
    // SimTK::Force::MobilityLinearStop lumbarForceY(m_forces, m_mobod[Torso],
    //         MobilizerQIndex(2), 500, 9.02585,
    //         convertDegreesToRadians(-30.0), convertDegreesToRadians(30.0));
    // SimTK::Force::MobilityLinearStop lumbarForceZ(m_forces, m_mobod[Torso],
    //         MobilizerQIndex(3), 500, 9.02585,
    //         convertDegreesToRadians(-60.0), convertDegreesToRadians(30.0));

    SimTK::Force::MobilityLinearStop leftHipForceX(m_forces, m_mobod[LeftThigh],
            MobilizerQIndex(1), 20, 1.22629,
            convertDegreesToRadians(-20.0), convertDegreesToRadians(45.0));
    SimTK::Force::MobilityLinearStop rightHipForceX(m_forces, m_mobod[RightThigh],
            MobilizerQIndex(1), 20, 1.22629,
            convertDegreesToRadians(-20.0), convertDegreesToRadians(45.0));
    SimTK::Force::MobilityLinearStop leftHipForceZ(m_forces, m_mobod[LeftThigh],
            MobilizerQIndex(3), 20, 1.22629,
            convertDegreesToRadians(-30.0), convertDegreesToRadians(120.0));
    SimTK::Force::MobilityLinearStop rightHipForceZ(m_forces, m_mobod[RightThigh],
            MobilizerQIndex(3), 20, 1.22629,
            convertDegreesToRadians(-30.0), convertDegreesToRadians(120.0));

    SimTK::Force::MobilityLinearStop leftKneeForce(m_forces, m_mobod[LeftShank],
            MobilizerQIndex(0), 500, 2.95953,
            convertDegreesToRadians(-120.0), convertDegreesToRadians(-3.0));
    SimTK::Force::MobilityLinearStop rightKneeForce(m_forces, m_mobod[RightShank],
            MobilizerQIndex(0), 500, 2.95953,
            convertDegreesToRadians(-120.0), convertDegreesToRadians(-3.0));

    SimTK::Force::MobilityLinearStop leftAnkleForce(m_forces, m_mobod[LeftFoot],
            MobilizerQIndex(0), 500, 1.41762,
            convertDegreesToRadians(-60.0), convertDegreesToRadians(25.0));
    SimTK::Force::MobilityLinearStop rightAnkleForce(m_forces, m_mobod[RightFoot],
            MobilizerQIndex(0), 500, 1.41762,
            convertDegreesToRadians(-60.0), convertDegreesToRadians(25.0));

    // Muscles
    // -------
    DecorativeLine baseLine;
    baseLine.setColor(Red).setLineThickness(4).setOpacity(.2);
    PointPathMuscle* muscle;

    // glut_med_r
    muscle = new PointPathMuscle(m_matter, 2045, 0.0733, 0.066, 0.3578);
    muscle->addPoint(m_mobod[Pelvis],     Vec3(-0.0148, 0.0445, 0.0766));
    muscle->addPoint(m_mobod[RightThigh], Vec3(-0.0258, 0.1642, 0.0527));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[GlutMed_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // add_mag_r
    muscle = new PointPathMuscle(m_matter, 2268, 0.087, 0.06, 0.0872665);
    muscle->addPoint(m_mobod[Pelvis],     Vec3(-0.0025, -0.1174, 0.0255));
    muscle->addPoint(m_mobod[RightThigh], Vec3(-0.0045, 0.0489, 0.0339));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[AddMag_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // hamstrings_r
    muscle = new PointPathMuscle(m_matter, 2594, 0.0976, 0.319, 0.2025);
    muscle->addPoint(m_mobod[Pelvis],     Vec3(-0.05526, -0.10257, 0.06944));
    muscle->addPoint(m_mobod[RightShank], Vec3(-0.028, 0.1667, 0.02943));
    muscle->addPoint(m_mobod[RightShank], Vec3(-0.021, 0.1467, 0.0343));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Hamstrings_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // bifemsh_r
    muscle = new PointPathMuscle(m_matter, 804, 0.1103, 0.095, 0.2147);
    muscle->addPoint(m_mobod[RightThigh], Vec3(0.005, -0.0411, 0.0234));
    muscle->addPoint(m_mobod[RightShank], Vec3(-0.028, 0.1667, 0.02943));
    muscle->addPoint(m_mobod[RightShank], Vec3(-0.021, 0.1467, 0.0343));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Bifemsh_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // glut_max_r
    muscle = new PointPathMuscle(m_matter, 1944, 0.1569, 0.111, 0.3822);
    muscle->addPoint(m_mobod[Pelvis],     Vec3(-0.0642, 0.0176, 0.0563));
    muscle->addPoint(m_mobod[Pelvis],     Vec3(-0.0669, -0.052, 0.0914));
    muscle->addPoint(m_mobod[RightThigh], Vec3(-0.0426, 0.117, 0.0293));
    muscle->addPoint(m_mobod[RightThigh], Vec3(-0.0156, 0.0684, 0.0419));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[GlutMax_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // iliopsoas_r
    muscle = new PointPathMuscle(m_matter, 2186, 0.1066, 0.152, 0.2496);
    muscle->addPoint(m_mobod[Pelvis],     Vec3(0.006, 0.0887, 0.0289));
    muscle->addPoint(m_mobod[Pelvis],     Vec3(0.0407, -0.01, 0.076));
    muscle->addPoint(m_mobod[RightThigh], Vec3(0.033, 0.135, 0.0038));
    muscle->addPoint(m_mobod[RightThigh], Vec3(-0.0188, 0.1103, 0.0104));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Iliopsoas_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // rect_fem_r
    muscle = new PointPathMuscle(m_matter, 1169, 0.0759, 0.3449, 0.2426);
    muscle->addPoint(m_mobod[Pelvis],     Vec3(0.0412, -0.0311, 0.0968));
    muscle->addPoint(m_mobod[RightThigh], Vec3(0.038, -0.17, 0.004));
    muscle->addPoint(m_mobod[RightShank], Vec3(0.038, 0.2117, 0.0018));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[RectFem_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // vasti_r
    muscle = new PointPathMuscle(m_matter, 4530, 0.0993, 0.1231, 0.0785);
    muscle->addPoint(m_mobod[RightThigh], Vec3(0.029, -0.0224, 0.031));
    muscle->addPoint(m_mobod[RightThigh], Vec3(0.038, -0.17, 0.007));
    muscle->addPoint(m_mobod[RightShank], Vec3(0.038, 0.2117, 0.0018));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Vasti_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // gastroc_r
    muscle = new PointPathMuscle(m_matter, 2241, 0.051, 0.384, 0.1728);
    muscle->addPoint(m_mobod[RightThigh], Vec3(-0.02, -0.218, -0.024));
    muscle->addPoint(m_mobod[RightFoot],  Vec3(-0.095, 0.001, -0.0053));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Gastroc_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // soleus_r
    muscle = new PointPathMuscle(m_matter, 3549, 0.044, 0.248, 0.4939);
    muscle->addPoint(m_mobod[RightShank], Vec3(-0.0024, 0.0334, 0.0071));
    muscle->addPoint(m_mobod[RightFoot],  Vec3(-0.095, 0.001, -0.0053));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Soleus_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // tib_ant_r
    muscle = new PointPathMuscle(m_matter, 1579, 0.0683, 0.243, 0.1676);
    muscle->addPoint(m_mobod[RightShank], Vec3(0.0179, 0.0243, 0.0115));
    muscle->addPoint(m_mobod[RightShank], Vec3(0.0329, -0.2084, -0.0177));
    muscle->addPoint(m_mobod[RightFoot],  Vec3(0.0166, -0.0122, -0.0305));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[TibAnt_R] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // glut_med_l
    muscle = new PointPathMuscle(m_matter, 2045, 0.0733, 0.066, 0.3578);
    muscle->addPoint(m_mobod[Pelvis],    Vec3(-0.0148, 0.0445, -0.0766));
    muscle->addPoint(m_mobod[LeftThigh], Vec3(-0.0258, 0.1642, -0.0527));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[GlutMed_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // add_mag_l
    muscle = new PointPathMuscle(m_matter, 2268, 0.087, 0.06, 0.0872665);
    muscle->addPoint(m_mobod[Pelvis],    Vec3(-0.0025, -0.1174, -0.0255));
    muscle->addPoint(m_mobod[LeftThigh], Vec3(-0.0045, 0.0489, -0.0339));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[AddMag_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // hamstrings_l
    muscle = new PointPathMuscle(m_matter, 2594, 0.0976, 0.319, 0.2025);
    muscle->addPoint(m_mobod[Pelvis],   Vec3(-0.05526, -0.10257, -0.06944));
    muscle->addPoint(m_mobod[LeftShank], Vec3(-0.028, 0.1667, -0.02943));
    muscle->addPoint(m_mobod[LeftShank], Vec3(-0.021, 0.1467, -0.0343));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Hamstrings_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // bifemsh_l
    muscle = new PointPathMuscle(m_matter, 804, 0.1103, 0.095, 0.2147);
    muscle->addPoint(m_mobod[LeftThigh], Vec3(0.005, -0.0411, -0.0234));
    muscle->addPoint(m_mobod[LeftShank], Vec3(-0.028, 0.1667, -0.02943));
    muscle->addPoint(m_mobod[LeftShank], Vec3(-0.021, 0.1467, -0.0343));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Bifemsh_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // glut_max_l
    muscle = new PointPathMuscle(m_matter, 1944, 0.1569, 0.111, 0.3822);
    muscle->addPoint(m_mobod[Pelvis],    Vec3(-0.0642, 0.0176, -0.0563));
    muscle->addPoint(m_mobod[Pelvis],    Vec3(-0.0669, -0.052, -0.0914));
    muscle->addPoint(m_mobod[LeftThigh], Vec3(-0.0426, 0.117, -0.0293));
    muscle->addPoint(m_mobod[LeftThigh], Vec3(-0.0156, 0.0684, -0.0419));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[GlutMax_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // iliopsoas_l
    muscle = new PointPathMuscle(m_matter, 2186, 0.1066, 0.152, 0.2496);
    muscle->addPoint(m_mobod[Pelvis],    Vec3(0.006, 0.0887, -0.0289));
    muscle->addPoint(m_mobod[Pelvis],    Vec3(0.0407, -0.01, -0.076));
    muscle->addPoint(m_mobod[LeftThigh], Vec3(0.033, 0.135, -0.0038));
    muscle->addPoint(m_mobod[LeftThigh], Vec3(-0.0188, 0.1103, -0.0104));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Iliopsoas_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // rect_fem_l
    muscle = new PointPathMuscle(m_matter, 1169, 0.0759, 0.3449, 0.2426);
    muscle->addPoint(m_mobod[Pelvis],    Vec3(0.0412, -0.0311, -0.0968));
    muscle->addPoint(m_mobod[LeftThigh], Vec3(0.038, -0.17, -0.004));
    muscle->addPoint(m_mobod[LeftShank], Vec3(0.038, 0.2117, -0.0018));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[RectFem_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // vasti_l
    muscle = new PointPathMuscle(m_matter, 4530, 0.0993, 0.1231, 0.0785);
    muscle->addPoint(m_mobod[LeftThigh], Vec3(0.029, -0.0224, -0.031));
    muscle->addPoint(m_mobod[LeftThigh], Vec3(0.038, -0.17, -0.007));
    muscle->addPoint(m_mobod[LeftShank], Vec3(0.038, 0.2117, -0.0018));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Vasti_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // gastroc_l
    muscle = new PointPathMuscle(m_matter, 2241, 0.051, 0.384, 0.1728);
    muscle->addPoint(m_mobod[LeftThigh], Vec3(-0.02, -0.218, 0.024));
    muscle->addPoint(m_mobod[LeftFoot],  Vec3(-0.095, 0.001, 0.0053));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Gastroc_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // soleus_l
    muscle = new PointPathMuscle(m_matter, 3549, 0.044, 0.248, 0.4939);
    muscle->addPoint(m_mobod[LeftShank], Vec3(-0.0024, 0.0334, -0.0071));
    muscle->addPoint(m_mobod[LeftFoot],  Vec3(-0.095, 0.001, 0.0053));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[Soleus_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);

    // tib_ant_l
    muscle = new PointPathMuscle(m_matter, 1579, 0.0683, 0.243, 0.1676);
    muscle->addPoint(m_mobod[LeftShank], Vec3(0.0179, 0.0243, -0.0115));
    muscle->addPoint(m_mobod[LeftShank], Vec3(0.0329, -0.2084, 0.0177));
    muscle->addPoint(m_mobod[LeftFoot],  Vec3(0.0166, -0.0122, 0.0305));
    muscle->addDecorativeLines(m_viz, baseLine);
    m_muscles[TibAnt_L] = muscle;
    SimTK::Force::Custom(m_forces, muscle);
}

void Gait3D::loadDefaultState(State& state) {
    const static Real hipAbductionAngle = 5*Pi/180;
    const static Real hipFlexionAngle = 15*Pi/180;
    const static Real kneeAngle = -60*Pi/180;
    const static Real ankleAngle = 20*Pi/180;

    m_mobod[Pelvis].setQToFitTranslation(state, Vec3(0,1.05,0));

    // m_mobod[Torso].setQToFitRotation(state, Rotation(-Pi/6, ZAxis));

    // m_mobod[LeftThigh].setOneQ(state, 1, hipAbductionAngle);
    // m_mobod[LeftThigh].setOneQ(state, 3, hipFlexionAngle);
    // m_mobod[LeftShank].setOneQ(state, 0, kneeAngle);
    // m_mobod[LeftFoot].setOneQ(state, 0, ankleAngle);

    // m_mobod[RightThigh].setOneQ(state, 1, -hipAbductionAngle);
    // m_mobod[RightThigh].setOneQ(state, 3, hipFlexionAngle);
    // m_mobod[RightShank].setOneQ(state, 0, kneeAngle);
    // m_mobod[RightFoot].setOneQ(state, 0, ankleAngle);

}


//////////////////////////////////////////////////////////////////////////

PointPathMuscle::PointPathMuscle(const SimbodyMatterSubsystem& matter,
                                 const Real& maxIsometricForce,
                                 const Real& optimalFiberLength,
                                 const Real& tendonSlackLength,
                                 const Real& pennationAngle) :
        m_matter(matter), m_maxIsometricForce(maxIsometricForce),
        m_optimalFiberLength(optimalFiberLength),
        m_tendonSlackLength(tendonSlackLength),
        m_cosPennationAngle(std::cos(pennationAngle)) {
    m_bodies.resize(0);
    m_stations.resize(0);
}

void PointPathMuscle::addPoint(MobilizedBodyIndex body, const Vec3& station) {
    m_bodies.push_back(body);
    m_stations.push_back(station);
}

Real PointPathMuscle::calcTendonForce(const Real& tendonLength) const {
    const Real tendonStrain = tendonLength > m_tendonSlackLength ?
            (tendonLength - m_tendonSlackLength) / m_tendonSlackLength : 0;
    return m_maxIsometricForce * tendonStrain * (cT1*tendonStrain + cT2);
}

Real PointPathMuscle::calcActiveForceLengthMultiplier(
        const Real& normalizedFiberLength) const {
    if (r1 < normalizedFiberLength && normalizedFiberLength < r2) {
        Real fiberStrain = normalizedFiberLength - 1.0;
        Real fiberStrainSquared = fiberStrain*fiberStrain;
        Real fiberStrainCubed = fiberStrainSquared*fiberStrain;
        return cL1*fiberStrainCubed + cL2*fiberStrainSquared + 1.0;
    } else {
        return 0;
    }
}

Real PointPathMuscle::calcActiveForceVelocityMultiplier(
        const Real& normalizedFiberVelocity) const {
    if (normalizedFiberVelocity >= 0) {
        return Fvmax*normalizedFiberVelocity + cV2 /
                   (cV2 + normalizedFiberVelocity);
    } else if (-1.0 < normalizedFiberVelocity && normalizedFiberVelocity < 0.0) {
        return cV1*(normalizedFiberVelocity + 1.0) /
                   (cV1 - normalizedFiberVelocity);
    } else {
        return 0;
    }
}

Real PointPathMuscle::calcPassiveForceLengthMultiplier(
        const Real& normalizedFiberLength) const {
    if (normalizedFiberLength > 1.0) {
        Real fiberStrain = normalizedFiberLength - 1.0;
        Real fiberStrainSquared = fiberStrain*fiberStrain;
        Real fiberStrainCubed = fiberStrainSquared*fiberStrain;
        return cP1*fiberStrainCubed + cP2*fiberStrainSquared;
    } else {
        return 0;
    }
}

Real PointPathMuscle::calcFiberForce(const Real& activation,
                                     const Real& normalizedFiberLength,
                                     const Real& normalizedFiberVelocity) const {
    const Real fl = calcActiveForceLengthMultiplier(normalizedFiberLength);
    const Real fv = calcActiveForceVelocityMultiplier(normalizedFiberVelocity);
    const Real fp = calcPassiveForceLengthMultiplier(normalizedFiberLength);
    const Real fd = m_fiberDamping * normalizedFiberVelocity;

    return m_maxIsometricForce * (activation*fl*fv + fp + fd)
                               * m_cosPennationAngle;
}

void PointPathMuscle::calcForce(const State& state,
                                Vector_<SpatialVec>& bodyForces,
                                Vector_<Vec3>& particleForces,
                                Vector& mobilityForces) const {
    Real length = 0;
    Real lengthDot = 0;
    Array_<UnitVec3> dirs(m_bodies.size()-1);
    Array_<Vec3> s_G(m_bodies.size());

    const MobilizedBody& body1 = getMobilizedBody(m_bodies[0]);
    const Transform& X_GB1 = body1.getBodyTransform(state);
    Vec3 s1_G = X_GB1.R() * m_stations[0];
    s_G[0] = s1_G;
    Vec3 p1_G = X_GB1.p() + s1_G;
    Vec3 v1_G = body1.findStationVelocityInGround(state, m_stations[0]);
    for (int i = 1; i < m_bodies.size(); ++i) {
        const MobilizedBody& body2 = getMobilizedBody(m_bodies[i]);
        const Transform& X_GB2 = body2.getBodyTransform(state);
        const Vec3 s2_G = X_GB2.R() * m_stations[i];
        s_G[i] = s2_G;
        const Vec3 p2_G = X_GB2.p() + s2_G;
        const Vec3 v2_G = body2.findStationVelocityInGround(state, m_stations[i]);
        const Vec3 r_G = p2_G - p1_G; // vector from point1 to point2
        const UnitVec3 dir(r_G);
        dirs[i-1] = dir;

        const Real dist = r_G.norm();  // distance between the points
        if( dist < SignificantReal ) return;
        length += dist;
        lengthDot += dot(v2_G - v1_G, dir); // relative velocity

        s1_G = s2_G;
        p1_G = p2_G;
        v1_G = v2_G;
    }

    // Rigid tendon.
    const Real fiberLength = length - m_tendonSlackLength;
    const Real normalizedFiberLength = fiberLength / m_optimalFiberLength;
    const Real normalizedFiberVelocity =
        lengthDot / (m_vmax * m_optimalFiberLength);

    const Real activation = getActivation(state);
    const Real fiberForce = calcFiberForce(activation,
        normalizedFiberLength, normalizedFiberVelocity);

    for (int i = 1; i < m_bodies.size(); ++i) {
        const Vec3 f_G = fiberForce * dirs[i-1];
        bodyForces[m_bodies[i-1]] +=  SpatialVec(s_G[i-1] % f_G, f_G);
        bodyForces[m_bodies[i]]   -=  SpatialVec(s_G[i] % f_G, f_G);
    }
}

Real PointPathMuscle::calcPotentialEnergy(const State& state) const {
    // TODO
    return 1.0;
}


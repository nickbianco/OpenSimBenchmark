#include <OpenSim/OpenSim.h>
#include <string>
#include "../../utilities/utilities.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static const char HELP[] =
R"(Create a 3D gait model.

Usage:
  gait3d <output> --joint=<joint> --muscle=<muscle> --obstacles=<obstacles>
  gait3d -h | --help

Options:
  -j <joint>,      --joint <joint>            Set the joint type.
  -m <muscle>,     --muscle <muscle>          Set the muscle type.
  -o <obstacles>,  --obstacles <obstacles>    Use wrap obstacles.
)";

// Enums
// -----
enum BodyType {LeftFoot=0, RightFoot, LeftShank, RightShank, LeftThigh,
               RightThigh, Pelvis, Torso};

enum MuscleType {GlutMed_R=0, AddMag_R, Hamstrings_R, Bifemsh_R, GlutMax_R,
                 Iliopsoas_R, RectFem_R, Vasti_R, Gastroc_R, Soleus_R, TibAnt_R,
                 GlutMed_L, AddMag_L, Hamstrings_L, Bifemsh_L, GlutMax_L,
                 Iliopsoas_L, RectFem_L, Vasti_L, Gastroc_L, Soleus_L, TibAnt_L};

enum Contact {Heel=0, LateralToe, MedialToe};

enum JointType {Custom=0, Ball};

// Data arrays
// -----------
SimTK::Real massData[] = {1.25, 1.25, 3.7075, 3.7075, 9.3014, 9.3014,
                          11.777, 34.2366};

SimTK::Vec3 inertiaData[] = {SimTK::Vec3(0.0014, 0.0039, 0.0041),
                             SimTK::Vec3(0.0014, 0.0039, 0.0041),
                             SimTK::Vec3(0.0504, 0.0051, 0.0511),
                             SimTK::Vec3(0.0504, 0.0051, 0.0511),
                             SimTK::Vec3(0.1339, 0.0351, 0.1412),
                             SimTK::Vec3(0.1339, 0.0351, 0.1412),
                             SimTK::Vec3(0.1028, 0.0871, 0.0579),
                             SimTK::Vec3(1.4745, 0.7555, 1.4314)};

SimTK::Vec3 leftContactPoints[] = {SimTK::Vec3(-0.085, -0.015, 0.005),
                                   SimTK::Vec3(0.0425, -0.03, -0.041),
                                   SimTK::Vec3(0.085, -0.03, 0.0275)};

SimTK::Vec3 rightContactPoints[] = {SimTK::Vec3(-0.085, -0.015, -0.005),
                                    SimTK::Vec3(0.0425, -0.03, 0.041),
                                    SimTK::Vec3(0.085, -0.03, -0.0275)};

// Muscle utilities
// ----------------
template<typename MuscleType>
MuscleType* createMuscle(Model& model, const std::string& name,
                        double maxIsometricForce, double optimalFiberLength,
                        double tendonSlackLength, double pennationAngle) {
    auto* muscle = new MuscleType();
    muscle->setName(name);
    muscle->set_max_isometric_force(maxIsometricForce);
    muscle->set_optimal_fiber_length(optimalFiberLength);
    muscle->set_tendon_slack_length(tendonSlackLength);
    muscle->set_pennation_angle_at_optimal(pennationAngle);
    muscle->set_ignore_tendon_compliance(true);
    muscle->set_ignore_activation_dynamics(true);
    muscle->set_default_activation(0.01);
    model.addForce(muscle);
    return muscle;
}

// Specialization for DeGrooteFregly2016Muscle
template<>
DeGrooteFregly2016Muscle* createMuscle<DeGrooteFregly2016Muscle>(
        Model& model, const std::string& name,
        double maxIsometricForce, double optimalFiberLength,
        double tendonSlackLength, double pennationAngle) {
    auto* muscle = new DeGrooteFregly2016Muscle();
    muscle->setName(name);
    muscle->set_max_isometric_force(maxIsometricForce);
    muscle->set_optimal_fiber_length(optimalFiberLength);
    muscle->set_tendon_slack_length(tendonSlackLength);
    muscle->set_pennation_angle_at_optimal(pennationAngle);
    muscle->set_ignore_tendon_compliance(true);
    muscle->set_ignore_activation_dynamics(false);
    muscle->set_default_activation(0.01);
    model.addForce(muscle);
    return muscle;
}

// Specialization for Millard2012EquilibriumMuscle
template<>
Millard2012EquilibriumMuscle* createMuscle<Millard2012EquilibriumMuscle>(
        Model& model, const std::string& name,
        double maxIsometricForce, double optimalFiberLength,
        double tendonSlackLength, double pennationAngle) {
    auto* muscle = new Millard2012EquilibriumMuscle();
    muscle->setName(name);
    muscle->set_max_isometric_force(maxIsometricForce);
    muscle->set_optimal_fiber_length(optimalFiberLength);
    muscle->set_tendon_slack_length(tendonSlackLength);
    muscle->set_pennation_angle_at_optimal(pennationAngle);
    muscle->set_ignore_tendon_compliance(true);
    muscle->set_ignore_activation_dynamics(false);
    muscle->set_default_activation(0.01);
    model.addForce(muscle);
    return muscle;
}

// Specialization for PathActuator
template<>
PathActuator* createMuscle<PathActuator>(
        Model& model, const std::string& name,
        double maxIsometricForce, double optimalFiberLength,
        double tendonSlackLength, double pennationAngle) {
    auto* muscle = new PathActuator();
    muscle->setName(name);
    muscle->set_optimal_force(maxIsometricForce);
    model.addForce(muscle);
    return muscle;
}

template<typename MuscleType>
void createMuscles(Model& model, OpenSim::Body* pelvis, OpenSim::Body* torso,
        OpenSim::Body* leftThigh, OpenSim::Body* leftShank,
        OpenSim::Body* leftFoot, OpenSim::Body* rightThigh,
        OpenSim::Body* rightShank, OpenSim::Body* rightFoot,
        bool useObstacles) {

    // Wrap obstacles
    // --------------
    if (useObstacles) {
        auto* glut_max_r_obstacle = new ContactEllipsoid(
                SimTK::Vec3(0.04, 0.04, 0.1), SimTK::Vec3(-0.04, -0.085, 0.09),
                SimTK::Vec3(-0.2, 0.5, 0.), *pelvis);
        glut_max_r_obstacle->setName("glut_max_r_obstacle");
        pelvis->addComponent(glut_max_r_obstacle);

        auto* glut_max_l_obstacle = new ContactEllipsoid(
                SimTK::Vec3(0.04, 0.04, 0.1), SimTK::Vec3(-0.04, -0.085, -0.09),
                SimTK::Vec3(0.2, -0.5, 0.), *pelvis);
        glut_max_l_obstacle->setName("glut_max_l_obstacle");
        pelvis->addComponent(glut_max_l_obstacle);

        auto* gastroc_r_obstacle = new ContactEllipsoid(
                SimTK::Vec3(0.065, 0.065, 0.2), SimTK::Vec3(-0.00861, 0.05, 0.),
                SimTK::Vec3(2.9672, 0.028, -1.478), *rightShank);
        gastroc_r_obstacle->setName("gastroc_r_obstacle");
        rightShank->addComponent(gastroc_r_obstacle);

        auto* gastroc_l_obstacle = new ContactEllipsoid(
                SimTK::Vec3(0.065, 0.065, 0.2), SimTK::Vec3(-0.00861, 0.05, 0.),
                SimTK::Vec3(-2.9672, 0.028, 1.478), *leftShank);
        gastroc_l_obstacle->setName("gastroc_l_obstacle");
        leftShank->addComponent(gastroc_l_obstacle);
    }

    // Right leg muscles
    // -----------------
    // glut_med_r
    auto* glut_med_r = createMuscle<MuscleType>(
            model, "glut_med_r", 2045, 0.0733, 0.066, 0.3578);
    glut_med_r->set_path(Scholz2015GeometryPath());
    auto& glut_med_r_path =
            glut_med_r->template updPath<Scholz2015GeometryPath>();
    glut_med_r_path.setName("glut_med_r_path");
    glut_med_r_path.setOrigin(*pelvis, SimTK::Vec3(-0.0148, 0.0445, 0.0766));
    glut_med_r_path.setInsertion(*rightThigh,
            SimTK::Vec3(-0.0258, 0.1642, 0.0527));

    // add_mag_r
    auto* add_mag_r = createMuscle<MuscleType>(
            model, "add_mag_r", 2268, 0.087, 0.06, 0.0872665);
    add_mag_r->set_path(Scholz2015GeometryPath());
    auto& add_mag_r_path =
            add_mag_r->template updPath<Scholz2015GeometryPath>();
    add_mag_r_path.setName("add_mag_r_path");
    add_mag_r_path.setOrigin(*pelvis, SimTK::Vec3(-0.0025, -0.1174, 0.0255));
    add_mag_r_path.setInsertion(*rightThigh,
            SimTK::Vec3(-0.0045, 0.0489, 0.0339));

    // hamstrings_r
    auto* hamstrings_r = createMuscle<MuscleType>(
            model, "hamstrings_r", 2594, 0.0976, 0.319, 0.2025);
    hamstrings_r->set_path(Scholz2015GeometryPath());
    auto& hamstrings_r_path =
            hamstrings_r->template updPath<Scholz2015GeometryPath>();
    hamstrings_r_path.setName("hamstrings_r_path");
    hamstrings_r_path.setOrigin(*pelvis,
            SimTK::Vec3(-0.05526, -0.10257, 0.06944));
    hamstrings_r_path.setInsertion(*rightShank,
            SimTK::Vec3(-0.021, 0.1467, 0.0343));
    hamstrings_r_path.addViaPoint(*rightShank,
            SimTK::Vec3(-0.028, 0.1667, 0.02943));

    // bifemsh_r
    auto* bifemsh_r = createMuscle<MuscleType>(
            model, "bifemsh_r", 804, 0.1103, 0.095, 0.2147);
    bifemsh_r->set_path(Scholz2015GeometryPath());
    auto& bifemsh_r_path =
            bifemsh_r->template updPath<Scholz2015GeometryPath>();
    bifemsh_r_path.setName("bifemsh_r_path");
    bifemsh_r_path.setOrigin(*rightThigh, SimTK::Vec3(0.005, -0.0411, 0.0234));
    bifemsh_r_path.setInsertion(*rightShank,
            SimTK::Vec3(-0.021, 0.1467, 0.0343));
    bifemsh_r_path.addViaPoint(*rightShank,
            SimTK::Vec3(-0.028, 0.1667, 0.02943));

    // glut_max_r
    auto* glut_max_r = createMuscle<MuscleType>(
            model, "glut_max_r", 1944, 0.1569, 0.111, 0.3822);
    glut_max_r->set_path(Scholz2015GeometryPath());
    auto& glut_max_r_path = glut_max_r->template updPath<Scholz2015GeometryPath>();
    glut_max_r_path.setName("glut_max_r_path");
    glut_max_r_path.setOrigin(*pelvis, SimTK::Vec3(-0.0642, 0.0176, 0.0563));
    glut_max_r_path.setInsertion(*rightThigh,
            SimTK::Vec3(-0.0156, 0.0684, 0.0419));
    if (useObstacles) {
        glut_max_r_path.addObstacle(
            pelvis->getComponent<OpenSim::ContactGeometry>("glut_max_r_obstacle"),
            SimTK::Vec3(-0.04, 0., 0.));
    } else {
        glut_max_r_path.addViaPoint(*pelvis,
            SimTK::Vec3(-0.0669, -0.052, 0.0914));
        glut_max_r_path.addViaPoint(*rightThigh,
            SimTK::Vec3(-0.0426, 0.117, 0.0293));
    }

    // iliopsoas_r
    auto* iliopsoas_r = createMuscle<MuscleType>(
            model, "iliopsoas_r", 2186, 0.1066, 0.152, 0.2496);
    iliopsoas_r->set_path(Scholz2015GeometryPath());
    auto& iliopsoas_r_path =
            iliopsoas_r->template updPath<Scholz2015GeometryPath>();
    iliopsoas_r_path.setName("iliopsoas_r_path");
    iliopsoas_r_path.setOrigin(*pelvis, SimTK::Vec3(0.006, 0.0887, 0.0289));
    iliopsoas_r_path.setInsertion(*rightThigh,
            SimTK::Vec3(-0.0188, 0.1103, 0.0104));
    iliopsoas_r_path.addViaPoint(*pelvis, SimTK::Vec3(0.0407, -0.01, 0.076));
    iliopsoas_r_path.addViaPoint(*rightThigh,
            SimTK::Vec3(0.033, 0.135, 0.0038));

    // rect_fem_r
    auto* rect_fem_r = createMuscle<MuscleType>(
            model, "rect_fem_r", 1169, 0.0759, 0.3449, 0.2426);
    rect_fem_r->set_path(Scholz2015GeometryPath());
    auto& rect_fem_r_path =
            rect_fem_r->template updPath<Scholz2015GeometryPath>();
    rect_fem_r_path.setName("rect_fem_r_path");
    rect_fem_r_path.setOrigin(*pelvis, SimTK::Vec3(0.0412, -0.0311, 0.0968));
    rect_fem_r_path.setInsertion(*rightShank,
            SimTK::Vec3(0.038, 0.2117, 0.0018));
    rect_fem_r_path.addViaPoint(*rightThigh, SimTK::Vec3(0.038, -0.17, 0.004));

    // vasti_r
    auto* vasti_r = createMuscle<MuscleType>(
            model, "vasti_r", 4530, 0.0993, 0.1231, 0.0785);
    vasti_r->set_path(Scholz2015GeometryPath());
    auto& vasti_r_path = vasti_r->template updPath<Scholz2015GeometryPath>();
    vasti_r_path.setName("vasti_r_path");
    vasti_r_path.setOrigin(*rightThigh, SimTK::Vec3(0.029, -0.0224, 0.031));
    vasti_r_path.setInsertion(*rightShank, SimTK::Vec3(0.038, 0.2117, 0.0018));
    vasti_r_path.addViaPoint(*rightThigh, SimTK::Vec3(0.038, -0.17, 0.007));

    // gastroc_r
    auto* gastroc_r = createMuscle<MuscleType>(
            model, "gastroc_r", 2241, 0.051, 0.384, 0.1728);
    gastroc_r->set_path(Scholz2015GeometryPath());
    auto& gastroc_r_path =
            gastroc_r->template updPath<Scholz2015GeometryPath>();
    gastroc_r_path.setName("gastroc_r_path");
    gastroc_r_path.setOrigin(*rightThigh, SimTK::Vec3(-0.02, -0.218, -0.024));
    gastroc_r_path.setInsertion(*rightFoot,
            SimTK::Vec3(-0.095, 0.001, -0.0053));
    if (useObstacles) {
        gastroc_r_path.addObstacle(
            rightShank->getComponent<OpenSim::ContactGeometry>("gastroc_r_obstacle"),
            SimTK::Vec3(0., -0.065, 0.));
    }

    // soleus_r
    auto* soleus_r = createMuscle<MuscleType>(
            model, "soleus_r", 3549, 0.044, 0.248, 0.4939);
    soleus_r->set_path(Scholz2015GeometryPath());
    auto& soleus_r_path = soleus_r->template updPath<Scholz2015GeometryPath>();
    soleus_r_path.setName("soleus_r_path");
    soleus_r_path.setOrigin(*rightShank, SimTK::Vec3(-0.0024, 0.0334, 0.0071));
    soleus_r_path.setInsertion(*rightFoot, SimTK::Vec3(-0.095, 0.001, -0.0053));

    // tib_ant_r
    auto* tib_ant_r = createMuscle<MuscleType>(
            model, "tib_ant_r", 1579, 0.0683, 0.243, 0.1676);
    tib_ant_r->set_path(Scholz2015GeometryPath());
    auto& tib_ant_r_path = tib_ant_r->template updPath<Scholz2015GeometryPath>();
    tib_ant_r_path.setName("tib_ant_r_path");
    tib_ant_r_path.setOrigin(*rightShank, SimTK::Vec3(0.0179, 0.0243, 0.0115));
    tib_ant_r_path.setInsertion(*rightFoot,
            SimTK::Vec3(0.0166, -0.0122, -0.0305));
    tib_ant_r_path.addViaPoint(*rightShank,
            SimTK::Vec3(0.0329, -0.2084, -0.0177));

    // Left leg muscles
    // -----------------
    auto* glut_med_l = createMuscle<MuscleType>(
            model, "glut_med_l", 2045, 0.0733, 0.066, 0.3578);
    glut_med_l->set_path(Scholz2015GeometryPath());
    auto& glut_med_l_path =
            glut_med_l->template updPath<Scholz2015GeometryPath>();
    glut_med_l_path.setName("glut_med_l_path");
    glut_med_l_path.setOrigin(*pelvis, SimTK::Vec3(-0.0148, 0.0445, -0.0766));
    glut_med_l_path.setInsertion(*leftThigh,
            SimTK::Vec3(-0.0258, 0.1642, -0.0527));

    // add_mag_l
    auto* add_mag_l = createMuscle<MuscleType>(
            model, "add_mag_l", 2268, 0.087, 0.06, 0.0872665);
    add_mag_l->set_path(Scholz2015GeometryPath());
    auto& add_mag_l_path =
            add_mag_l->template updPath<Scholz2015GeometryPath>();
    add_mag_l_path.setName("add_mag_l_path");
    add_mag_l_path.setOrigin(*pelvis, SimTK::Vec3(-0.0025, -0.1174, -0.0255));
    add_mag_l_path.setInsertion(*leftThigh,
            SimTK::Vec3(-0.0045, 0.0489, -0.0339));

    // hamstrings_l
    auto* hamstrings_l = createMuscle<MuscleType>(
            model, "hamstrings_l", 2594, 0.0976, 0.319, 0.2025);
    hamstrings_l->set_path(Scholz2015GeometryPath());
    auto& hamstrings_l_path =
            hamstrings_l->template updPath<Scholz2015GeometryPath>();
    hamstrings_l_path.setName("hamstrings_l_path");
    hamstrings_l_path.setOrigin(*pelvis,
            SimTK::Vec3(-0.05526, -0.10257, -0.06944));
    hamstrings_l_path.setInsertion(*leftShank,
            SimTK::Vec3(-0.021, 0.1467, -0.0343));
    hamstrings_l_path.addViaPoint(*leftShank,
            SimTK::Vec3(-0.028, 0.1667, -0.02943));

    // bifemsh_l
    auto* bifemsh_l = createMuscle<MuscleType>(
            model, "bifemsh_l", 804, 0.1103, 0.095, 0.2147);
    bifemsh_l->set_path(Scholz2015GeometryPath());
    auto& bifemsh_l_path =
            bifemsh_l->template updPath<Scholz2015GeometryPath>();
    bifemsh_l_path.setName("bifemsh_l_path");
    bifemsh_l_path.setOrigin(*leftThigh, SimTK::Vec3(0.005, -0.0411, -0.0234));
    bifemsh_l_path.setInsertion(*leftShank,
            SimTK::Vec3(-0.021, 0.1467, -0.0343));
    bifemsh_l_path.addViaPoint(*leftShank,
            SimTK::Vec3(-0.028, 0.1667, -0.02943));

    // glut_max_l
    auto* glut_max_l = createMuscle<MuscleType>(
            model, "glut_max_l", 1944, 0.1569, 0.111, 0.3822);
    glut_max_l->set_path(Scholz2015GeometryPath());
    auto& glut_max_l_path =
            glut_max_l->template updPath<Scholz2015GeometryPath>();
    glut_max_l_path.setName("glut_max_l_path");
    glut_max_l_path.setOrigin(*pelvis, SimTK::Vec3(-0.0642, 0.0176, -0.0563));
    glut_max_l_path.setInsertion(*leftThigh,
            SimTK::Vec3(-0.0156, 0.0684, -0.0419));
    if (useObstacles) {
        glut_max_l_path.addObstacle(
            pelvis->getComponent<OpenSim::ContactGeometry>("glut_max_l_obstacle"),
            SimTK::Vec3(-0.04, 0., 0.));
    } else {
        glut_max_l_path.addViaPoint(*pelvis,
            SimTK::Vec3(-0.0669, -0.052, -0.0914));
        glut_max_l_path.addViaPoint(*leftThigh,
            SimTK::Vec3(-0.0426, 0.117, -0.0293));
    }

    // iliopsoas_l
    auto* iliopsoas_l = createMuscle<MuscleType>(
            model, "iliopsoas_l", 2186, 0.1066, 0.152, 0.2496);
    iliopsoas_l->set_path(Scholz2015GeometryPath());
    auto& iliopsoas_l_path =
            iliopsoas_l->template updPath<Scholz2015GeometryPath>();
    iliopsoas_l_path.setName("iliopsoas_l_path");
    iliopsoas_l_path.setOrigin(*pelvis, SimTK::Vec3(0.006, 0.0887, -0.0289));
    iliopsoas_l_path.setInsertion(*leftThigh,
            SimTK::Vec3(-0.0188, 0.1103, -0.0104));
    iliopsoas_l_path.addViaPoint(*pelvis, SimTK::Vec3(0.0407, -0.01, -0.076));
    iliopsoas_l_path.addViaPoint(*leftThigh,
            SimTK::Vec3(0.033, 0.135, -0.0038));

    // rect_fem_l
    auto* rect_fem_l = createMuscle<MuscleType>(
            model, "rect_fem_l", 1169, 0.0759, 0.3449, 0.2426);
    rect_fem_l->set_path(Scholz2015GeometryPath());
    auto& rect_fem_l_path =
            rect_fem_l->template updPath<Scholz2015GeometryPath>();
    rect_fem_l_path.setName("rect_fem_l_path");
    rect_fem_l_path.setOrigin(*pelvis, SimTK::Vec3(0.0412, -0.0311, -0.0968));
    rect_fem_l_path.setInsertion(*leftShank,
            SimTK::Vec3(0.038, 0.2117, -0.0018));
    rect_fem_l_path.addViaPoint(*leftThigh, SimTK::Vec3(0.038, -0.17, -0.004));

    // vasti_l
    auto* vasti_l = createMuscle<MuscleType>(
            model, "vasti_l", 4530, 0.0993, 0.1231, 0.0785);
    vasti_l->set_path(Scholz2015GeometryPath());
    auto& vasti_l_path = vasti_l->template updPath<Scholz2015GeometryPath>();
    vasti_l_path.setName("vasti_l_path");
    vasti_l_path.setOrigin(*leftThigh, SimTK::Vec3(0.029, -0.0224, -0.031));
    vasti_l_path.setInsertion(*leftShank, SimTK::Vec3(0.038, 0.2117, -0.0018));
    vasti_l_path.addViaPoint(*leftThigh, SimTK::Vec3(0.038, -0.17, -0.007));

    // gastroc_l
    auto* gastroc_l = createMuscle<MuscleType>(
            model, "gastroc_l", 2241, 0.051, 0.384, 0.1728);
    gastroc_l->set_path(Scholz2015GeometryPath());
    auto& gastroc_l_path = gastroc_l->template updPath<Scholz2015GeometryPath>();
    gastroc_l_path.setName("gastroc_l_path");
    gastroc_l_path.setOrigin(*leftThigh, SimTK::Vec3(-0.02, -0.218, 0.024));
    gastroc_l_path.setInsertion(*leftFoot, SimTK::Vec3(-0.095, 0.001, 0.0053));
    if (useObstacles) {
        gastroc_l_path.addObstacle(
            leftShank->getComponent<OpenSim::ContactGeometry>("gastroc_l_obstacle"),
            SimTK::Vec3(-0.065, 0., 0.));
    }

    // soleus_l
    auto* soleus_l = createMuscle<MuscleType>(
            model, "soleus_l", 3549, 0.044, 0.248, 0.4939);
    soleus_l->set_path(Scholz2015GeometryPath());
    auto& soleus_l_path = soleus_l->template updPath<Scholz2015GeometryPath>();
    soleus_l_path.setName("soleus_l_path");
    soleus_l_path.setOrigin(*leftShank, SimTK::Vec3(-0.0024, 0.0334, -0.0071));
    soleus_l_path.setInsertion(*leftFoot, SimTK::Vec3(-0.095, 0.001, 0.0053));

    // tib_ant_l
    auto* tib_ant_l = createMuscle<MuscleType>(
            model, "tib_ant_l", 1579, 0.0683, 0.243, 0.1676);
    tib_ant_l->set_path(Scholz2015GeometryPath());
    auto& tib_ant_l_path =
            tib_ant_l->template updPath<Scholz2015GeometryPath>();
    tib_ant_l_path.setName("tib_ant_l_path");
    tib_ant_l_path.setOrigin(*leftShank, SimTK::Vec3(0.0179, 0.0243, -0.0115));
    tib_ant_l_path.setInsertion(*leftFoot,
            SimTK::Vec3(0.0166, -0.0122, -0.0305));
    tib_ant_l_path.addViaPoint(*leftShank,
            SimTK::Vec3(0.0329, -0.2084, -0.0177));
}

// Contact utilities
// ----------------
void addContactGeometry(OpenSim::Body* body, const SimTK::Vec3& offset,
                       const std::string& name, const double& radius) {
    PhysicalOffsetFrame* offsetFrame =
            new PhysicalOffsetFrame(name, *body, offset);
    offsetFrame->attachGeometry(new Sphere(radius));
    body->addComponent(offsetFrame);
}

void addContact(Model& model, const std::string& name, PhysicalFrame* frame,
               const SimTK::Vec3& location, const SimTK::Transform& transform,
               const SimTK::ExponentialSpringParameters& params) {
    auto* contact = new ExponentialContactForce(transform, *frame, location,
        params);
    contact->setName(name);
    model.addForce(contact);
}

int main(int argc, char* argv[]) {

    // Parse the command line arguments.
    // ---------------------------------
    std::map<std::string, docopt::value> args = parse_arguments(
            HELP, { argv + 1, argv + argc }, true);

    // Muscle model
    std::string muscleModel = "millard";
    if (args["--muscle"]) {
        muscleModel = args["--muscle"].asString();
        OPENSIM_THROW_IF(muscleModel != "degrootefregly" &&
                        muscleModel != "millard" &&
                        muscleModel != "pathactuator",
                        OpenSim::Exception,
                        "Invalid muscle model specified. Must be one of: "
                        "'degrootefregly', 'millard', 'pathactuator'.");
    }

    // Joint type
    std::string joint = "custom";
    JointType jointType = JointType::Custom;
    if (args["--joint"]) {
        joint = args["--joint"].asString();
        OPENSIM_THROW_IF(joint != "custom" &&
                        joint != "ball",
                        OpenSim::Exception,
                        "Invalid joint type specified. Must be one of: "
                        "'custom', 'ball'.");
    }
    if (joint == "custom") {
        jointType = JointType::Custom;
    } else if (joint == "ball") {
        jointType = JointType::Ball;
    }

    // Wrap obstacles
    // ---------------
    bool useObstacles = false;
    if (args["--obstacles"]) {
        std::string obstacles = args["--obstacles"].asString();
        useObstacles = (obstacles == "true");
    }

    // Parameters
    // ----------
    SimTK::Real jointDamping = 1.0;
    SimTK::Real stopStiffness = 500.0;
    SimTK::Real stopDamping = 2.95953;

    // Create the model.
    // ----------------
    Model model;

    // Bodies
    // ------
    // pelvis
    OpenSim::Body* pelvis = new OpenSim::Body("pelvis", massData[Pelvis],
            SimTK::Vec3(0), SimTK::Inertia(inertiaData[Pelvis]));
    PhysicalOffsetFrame* pelvisOffset = new PhysicalOffsetFrame("pelvis_offset",
            *pelvis, SimTK::Vec3(-0.01, -0.05, 0));
    pelvisOffset->attachGeometry(new Ellipsoid(0.07, 0.07, 0.12));
    pelvis->addComponent(pelvisOffset);

    // torso
    OpenSim::Body* torso = new OpenSim::Body("torso", massData[Torso],
            SimTK::Vec3(0), SimTK::Inertia(inertiaData[Torso]));
    torso->attachGeometry(new Ellipsoid(0.1, 0.27, 0.1));
    addContactGeometry(torso, SimTK::Vec3(0, 0.38, 0), "torso_offset", 0.09);

    // left thigh
    OpenSim::Body* leftThigh = new OpenSim::Body("leftThigh",
            massData[LeftThigh], SimTK::Vec3(0),
            SimTK::Inertia(inertiaData[LeftThigh]));
    leftThigh->attachGeometry(new Ellipsoid(0.04, 0.2, 0.04));

    // left shank
    OpenSim::Body* leftShank = new OpenSim::Body("leftShank",
            massData[LeftShank], SimTK::Vec3(0),
            SimTK::Inertia(inertiaData[LeftShank]));
    leftShank->attachGeometry(new Cylinder(0.02, 0.22));

    // left foot
    OpenSim::Body* leftFoot = new OpenSim::Body("leftFoot", massData[LeftFoot],
            SimTK::Vec3(0), SimTK::Inertia(inertiaData[LeftFoot]));
    leftFoot->attachGeometry(new Ellipsoid(0.1, 0.03, 0.05));
    addContactGeometry(leftFoot, leftContactPoints[0], "heel_geometry", 0.01);
    addContactGeometry(leftFoot, leftContactPoints[1],
            "lateralToe_geometry", 0.01);
    addContactGeometry(leftFoot, leftContactPoints[2],
            "medialToe_geometry", 0.01);

    // right thigh
    OpenSim::Body* rightThigh = new OpenSim::Body("rightThigh",
            massData[RightThigh], SimTK::Vec3(0),
            SimTK::Inertia(inertiaData[RightThigh]));
    rightThigh->attachGeometry(new Ellipsoid(0.04, 0.2, 0.04));

    // right shank
    OpenSim::Body* rightShank = new OpenSim::Body("rightShank",
            massData[RightShank], SimTK::Vec3(0),
            SimTK::Inertia(inertiaData[RightShank]));
    rightShank->attachGeometry(new Cylinder(0.02, 0.22));

    // right foot
    OpenSim::Body* rightFoot = new OpenSim::Body("rightFoot",
            massData[RightFoot], SimTK::Vec3(0),
            SimTK::Inertia(inertiaData[RightFoot]));
    rightFoot->attachGeometry(new Ellipsoid(0.1, 0.03, 0.05));
    addContactGeometry(rightFoot, rightContactPoints[0], "heel_geometry", 0.01);
    addContactGeometry(rightFoot, rightContactPoints[1],
            "lateralToe_geometry", 0.01);
    addContactGeometry(rightFoot, rightContactPoints[2],
            "medialToe_geometry", 0.01);

    model.addBody(pelvis);
    model.addBody(torso);
    model.addBody(leftThigh);
    model.addBody(leftShank);
    model.addBody(leftFoot);
    model.addBody(rightThigh);
    model.addBody(rightShank);
    model.addBody(rightFoot);

    // Joints
    // ------
    FreeJoint* pelvisGround = new FreeJoint("pelvis_ground",
            model.getGround(), *pelvis);

    Joint* lumbar;
    Joint* leftHip;
    Joint* rightHip;
    if (jointType == JointType::Custom) {
        SpatialTransform lumbar_transform;
        lumbar_transform[0].setCoordinateNames(
                OpenSim::Array<std::string>("lumbar_coord_0", 1, 1));
        lumbar_transform[0].setFunction(new LinearFunction());
        lumbar_transform[1].setCoordinateNames(
                OpenSim::Array<std::string>("lumbar_coord_1", 1, 1));
        lumbar_transform[1].setFunction(new LinearFunction());
        lumbar_transform[2].setCoordinateNames(
                OpenSim::Array<std::string>("lumbar_coord_2", 1, 1));
        lumbar_transform[2].setFunction(new LinearFunction());
        lumbar = new CustomJoint("lumbar",
            *pelvis, SimTK::Vec3(0, 0.05, 0), SimTK::Vec3(0),
            *torso, SimTK::Vec3(0, -0.25, 0), SimTK::Vec3(0),
            lumbar_transform);

        SpatialTransform hip_l_transform;
        hip_l_transform[0].setCoordinateNames(
                OpenSim::Array<std::string>("hip_l_coord_0", 1, 1));
        hip_l_transform[0].setFunction(new LinearFunction());
        hip_l_transform[1].setCoordinateNames(
                OpenSim::Array<std::string>("hip_l_coord_1", 1, 1));
        hip_l_transform[1].setFunction(new LinearFunction());
        hip_l_transform[2].setCoordinateNames(
                OpenSim::Array<std::string>("hip_l_coord_2", 1, 1));
        hip_l_transform[2].setFunction(new LinearFunction());
        leftHip = new CustomJoint("hip_l",
            *pelvis, SimTK::Vec3(0, -0.0661, -0.0835), SimTK::Vec3(0),
            *leftThigh, SimTK::Vec3(0, 0.17, 0), SimTK::Vec3(0),
            hip_l_transform);

        SpatialTransform hip_r_transform;
        hip_r_transform[0].setCoordinateNames(
                OpenSim::Array<std::string>("hip_r_coord_0", 1, 1));
        hip_r_transform[0].setFunction(new LinearFunction());
        hip_r_transform[1].setCoordinateNames(
                OpenSim::Array<std::string>("hip_r_coord_1", 1, 1));
        hip_r_transform[1].setFunction(new LinearFunction());
        hip_r_transform[2].setCoordinateNames(
                OpenSim::Array<std::string>("hip_r_coord_2", 1, 1));
        hip_r_transform[2].setFunction(new LinearFunction());
        rightHip = new CustomJoint("hip_r",
            *pelvis, SimTK::Vec3(0, -0.0661, 0.0835), SimTK::Vec3(0),
            *rightThigh, SimTK::Vec3(0, 0.17, 0), SimTK::Vec3(0),
            hip_r_transform);
    } else {
        lumbar = new BallJoint("lumbar",
            *pelvis, SimTK::Vec3(0, 0.05, 0), SimTK::Vec3(0),
            *torso, SimTK::Vec3(0, -0.25, 0), SimTK::Vec3(0));

        leftHip = new BallJoint("hip_l",
            *pelvis, SimTK::Vec3(0, -0.0661, -0.0835), SimTK::Vec3(0),
            *leftThigh, SimTK::Vec3(0, 0.17, 0), SimTK::Vec3(0));

        rightHip = new BallJoint("hip_r",
            *pelvis, SimTK::Vec3(0, -0.0661, 0.0835), SimTK::Vec3(0),
            *rightThigh, SimTK::Vec3(0, 0.17, 0), SimTK::Vec3(0));
    }

    PinJoint* leftKnee = new PinJoint("knee_l",
            *leftThigh, SimTK::Vec3(0, -0.226, 0), SimTK::Vec3(0),
            *leftShank, SimTK::Vec3(0, 0.1867, 0), SimTK::Vec3(0));

    PinJoint* leftAnkle = new PinJoint("ankle_l",
            *leftShank, SimTK::Vec3(0, -0.2433, 0), SimTK::Vec3(0),
            *leftFoot, SimTK::Vec3(-0.05123, 0.01195, 0.00792), SimTK::Vec3(0));

    PinJoint* rightKnee = new PinJoint("knee_r",
            *rightThigh, SimTK::Vec3(0, -0.226, 0), SimTK::Vec3(0),
            *rightShank, SimTK::Vec3(0, 0.1867, 0), SimTK::Vec3(0));

    PinJoint* rightAnkle = new PinJoint("ankle_r",
            *rightShank, SimTK::Vec3(0, -0.2433, 0), SimTK::Vec3(0),
            *rightFoot, SimTK::Vec3(-0.05123, 0.01195, -0.00792), SimTK::Vec3(0));

    model.addJoint(pelvisGround);
    model.addJoint(lumbar);
    model.addJoint(leftHip);
    model.addJoint(leftKnee);
    model.addJoint(leftAnkle);
    model.addJoint(rightHip);
    model.addJoint(rightKnee);
    model.addJoint(rightAnkle);

    // Muscles
    // -------
    if (muscleModel == "millard") {
        createMuscles<Millard2012EquilibriumMuscle>(model, pelvis, torso,
            leftThigh, leftShank, leftFoot, rightThigh, rightShank, rightFoot,
            useObstacles);
    } else if (muscleModel == "pathactuator") {
        createMuscles<PathActuator>(model, pelvis, torso, leftThigh,
            leftShank, leftFoot, rightThigh, rightShank, rightFoot,
            useObstacles);
    } else if (muscleModel == "degrootefregly") {
        createMuscles<DeGrooteFregly2016Muscle>(model, pelvis, torso, leftThigh,
            leftShank, leftFoot, rightThigh, rightShank, rightFoot,
            useObstacles);
    }

    // Lumbar torque actuators
    // -----------------------
    ActivationCoordinateActuator* lumbarTorqueX =
        new ActivationCoordinateActuator("lumbar_coord_0");
    lumbarTorqueX->setName("lumbar_coord_0_torque");
    lumbarTorqueX->setOptimalForce(100.0);
    lumbarTorqueX->setMinControl(-1.0);
    lumbarTorqueX->setMaxControl(1.0);
    lumbarTorqueX->set_activation_time_constant(0.05);
    model.addForce(lumbarTorqueX);

    ActivationCoordinateActuator* lumbarTorqueY =
        new ActivationCoordinateActuator("lumbar_coord_1");
    lumbarTorqueY->setName("lumbar_coord_1_torque");
    lumbarTorqueY->setOptimalForce(100.0);
    lumbarTorqueY->setMinControl(-1.0);
    lumbarTorqueY->setMaxControl(1.0);
    lumbarTorqueY->set_activation_time_constant(0.05);
    model.addForce(lumbarTorqueY);

    ActivationCoordinateActuator* lumbarTorqueZ =
        new ActivationCoordinateActuator("lumbar_coord_2");
    lumbarTorqueZ->setName("lumbar_coord_2_torque");
    lumbarTorqueZ->setOptimalForce(100.0);
    lumbarTorqueZ->setMinControl(-1.0);
    lumbarTorqueZ->setMaxControl(1.0);
    lumbarTorqueZ->set_activation_time_constant(0.05);
    model.addForce(lumbarTorqueZ);

    // Joint damping
    // -------------
    CoordinateLinearDamper* lumbarDamperX = new CoordinateLinearDamper(
            "lumbar_coord_0", jointDamping);
    lumbarDamperX->setName("lumbar_coord_0_damper");
    model.addForce(lumbarDamperX);

    CoordinateLinearDamper* lumbarDamperY = new CoordinateLinearDamper(
            "lumbar_coord_1", jointDamping);
    lumbarDamperY->setName("lumbar_coord_1_damper");
    model.addForce(lumbarDamperY);

    CoordinateLinearDamper* lumbarDamperZ = new CoordinateLinearDamper(
            "lumbar_coord_2", jointDamping);
    lumbarDamperZ->setName("lumbar_coord_2_damper");
    model.addForce(lumbarDamperZ);

    CoordinateLinearDamper* leftHipDamperX = new CoordinateLinearDamper(
            "hip_l_coord_0", jointDamping);
    leftHipDamperX->setName("hip_l_coord_0_damper");
    model.addForce(leftHipDamperX);

    CoordinateLinearDamper* leftHipDamperY = new CoordinateLinearDamper(
            "hip_l_coord_1", jointDamping);
    leftHipDamperY->setName("hip_l_coord_1_damper");
    model.addForce(leftHipDamperY);

    CoordinateLinearDamper* leftHipDamperZ = new CoordinateLinearDamper(
            "hip_l_coord_2", jointDamping);
    leftHipDamperZ->setName("hip_l_coord_2_damper");
    model.addForce(leftHipDamperZ);

    CoordinateLinearDamper* rightHipDamperX = new CoordinateLinearDamper(
            "hip_r_coord_0", jointDamping);
    rightHipDamperX->setName("hip_r_coord_0_damper");
    model.addForce(rightHipDamperX);

    CoordinateLinearDamper* rightHipDamperY = new CoordinateLinearDamper(
            "hip_r_coord_1", jointDamping);
    rightHipDamperY->setName("hip_r_coord_1_damper");
    model.addForce(rightHipDamperY);

    CoordinateLinearDamper* rightHipDamperZ = new CoordinateLinearDamper(
            "hip_r_coord_2", jointDamping);
    rightHipDamperZ->setName("hip_r_coord_2_damper");
    model.addForce(rightHipDamperZ);

    CoordinateLinearDamper* leftKneeDamper = new CoordinateLinearDamper(
            "knee_l_coord_0", jointDamping);
    leftKneeDamper->setName("knee_l_coord_0_damper");
    model.addForce(leftKneeDamper);

    CoordinateLinearDamper* leftAnkleDamper = new CoordinateLinearDamper(
            "ankle_l_coord_0", jointDamping);
    leftAnkleDamper->setName("ankle_l_coord_0_damper");
    model.addForce(leftAnkleDamper);

    CoordinateLinearDamper* rightKneeDamper = new CoordinateLinearDamper(
            "knee_r_coord_0", jointDamping);
    rightKneeDamper->setName("knee_r_coord_0_damper");
    model.addForce(rightKneeDamper);

    CoordinateLinearDamper* rightAnkleDamper = new CoordinateLinearDamper(
            "ankle_r_coord_0", jointDamping);
    rightAnkleDamper->setName("ankle_r_coord_0_damper");
    model.addForce(rightAnkleDamper);

    // Joint stops
    // ----------
    if (jointType == JointType::Custom) {
        CoordinateLinearStop* lumbarStop0 = new CoordinateLinearStop(
                "lumbar_coord_0", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-20.0),
                SimTK::convertDegreesToRadians(20.0));
        lumbarStop0->setName("lumbar_coord_0_stop");
        model.addForce(lumbarStop0);

        CoordinateLinearStop* lumbarStop1 = new CoordinateLinearStop(
                "lumbar_coord_1", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-20.0),
                SimTK::convertDegreesToRadians(20.0));
        lumbarStop1->setName("lumbar_coord_1_stop");
        model.addForce(lumbarStop1);

        CoordinateLinearStop* lumbarStop2 = new CoordinateLinearStop(
                "lumbar_coord_2", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-20.0),
                SimTK::convertDegreesToRadians(20.0));
        lumbarStop2->setName("lumbar_coord_2_stop");
        model.addForce(lumbarStop2);

        CoordinateLinearStop* leftHipStop0 = new CoordinateLinearStop(
                "hip_l_coord_0", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-30.0),
                SimTK::convertDegreesToRadians(30.0));
        leftHipStop0->setName("hip_l_coord_0_stop");
        model.addForce(leftHipStop0);

        CoordinateLinearStop* leftHipStop1 = new CoordinateLinearStop(
                "hip_l_coord_1", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-30.0),
                SimTK::convertDegreesToRadians(30.0));
        leftHipStop1->setName("hip_l_coord_1_stop");
        model.addForce(leftHipStop1);

        CoordinateLinearStop* leftHipStop2 = new CoordinateLinearStop(
                "hip_l_coord_2", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-30.0),
                SimTK::convertDegreesToRadians(30.0));
        leftHipStop2->setName("hip_l_coord_2_stop");
        model.addForce(leftHipStop2);

        CoordinateLinearStop* rightHipStop0 = new CoordinateLinearStop(
                "hip_r_coord_0", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-30.0),
                SimTK::convertDegreesToRadians(30.0));
        rightHipStop0->setName("hip_r_coord_0_stop");
        model.addForce(rightHipStop0);

        CoordinateLinearStop* rightHipStop1 = new CoordinateLinearStop(
                "hip_r_coord_1", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-30.0),
                SimTK::convertDegreesToRadians(30.0));
        rightHipStop1->setName("hip_r_coord_1_stop");
        model.addForce(rightHipStop1);

        CoordinateLinearStop* rightHipStop2 = new CoordinateLinearStop(
                "hip_r_coord_2", stopStiffness, stopDamping,
                SimTK::convertDegreesToRadians(-30.0),
                SimTK::convertDegreesToRadians(30.0));
        rightHipStop2->setName("hip_r_coord_2_stop");
        model.addForce(rightHipStop2);
    }

    CoordinateLinearStop* leftKneeStop = new CoordinateLinearStop(
            "knee_l_coord_0", stopStiffness, stopDamping,
            SimTK::convertDegreesToRadians(-120.0),
            SimTK::convertDegreesToRadians(-3.0));
    leftKneeStop->setName("knee_l_coord_0_stop");
    model.addForce(leftKneeStop);

    CoordinateLinearStop* rightKneeStop = new CoordinateLinearStop(
            "knee_r_coord_0", stopStiffness, stopDamping,
            SimTK::convertDegreesToRadians(-120.0),
            SimTK::convertDegreesToRadians(-3.0));
    rightKneeStop->setName("knee_r_coord_0_stop");
    model.addForce(rightKneeStop);

    CoordinateLinearStop* leftAnkleStopX = new CoordinateLinearStop(
            "ankle_l_coord_0", stopStiffness, stopDamping,
            SimTK::convertDegreesToRadians(-60.0),
            SimTK::convertDegreesToRadians(25.0));
    leftAnkleStopX->setName("ankle_l_coord_0_stop");
    model.addForce(leftAnkleStopX);

    CoordinateLinearStop* rightAnkleStopX = new CoordinateLinearStop(
            "ankle_r_coord_0", stopStiffness, stopDamping,
            SimTK::convertDegreesToRadians(-60.0),
            SimTK::convertDegreesToRadians(25.0));
    rightAnkleStopX->setName("ankle_r_coord_0_stop");
    model.addForce(rightAnkleStopX);

    // Contact
    // -------
    SimTK::Transform transform(SimTK::Rotation(-0.5*SimTK::Pi, SimTK::XAxis),
            SimTK::Vec3(0));
    SimTK::ExponentialSpringParameters params;
    params.setNormalViscosity(1.0);
    params.setInitialMuStatic(0.9);
    params.setInitialMuKinetic(0.6);
    params.setSettleVelocity(0.1);

    addContact(model, "left_heel_contact", leftFoot,
               leftContactPoints[0], transform, params);
    addContact(model, "left_lateralToe_contact", leftFoot,
               leftContactPoints[1], transform, params);
    addContact(model, "left_medialToe_contact", leftFoot,
               leftContactPoints[2], transform, params);

    addContact(model, "right_heel_contact", rightFoot,
               rightContactPoints[0], transform, params);
    addContact(model, "right_lateralToe_contact", rightFoot,
               rightContactPoints[1], transform, params);
    addContact(model, "right_medialToe_contact", rightFoot,
               rightContactPoints[2], transform, params);

    // Torso contact points
    addContact(model, "torso_front_contact", torso,
               SimTK::Vec3(0, 0.1, 0), transform, params);
    addContact(model, "torso_back_contacpt", torso,
               SimTK::Vec3(0, -0.1, 0), transform, params);
    addContact(model, "torso_left_contact", torso,
               SimTK::Vec3(0, 0, 0.05), transform, params);
    addContact(model, "torso_right_contact", torso,
               SimTK::Vec3(0, 0, -0.05), transform, params);

    // Pelvis contact points
    addContact(model, "pelvis_front_contact", pelvis,
               SimTK::Vec3(0, 0.05, 0), transform, params);
    addContact(model, "pelvis_back_contact", pelvis,
               SimTK::Vec3(0, -0.05, 0), transform, params);
    addContact(model, "pelvis_left_contact", pelvis,
               SimTK::Vec3(0, 0, 0.06), transform, params);
    addContact(model, "pelvis_right_contact", pelvis,
               SimTK::Vec3(0, 0, -0.06), transform, params);

    // Left hip contact points
    addContact(model, "left_hip_front_contact", leftThigh,
               SimTK::Vec3(0, 0.15, 0), transform, params);
    addContact(model, "left_hip_back_contact", leftThigh,
               SimTK::Vec3(0, 0.19, 0), transform, params);
    addContact(model, "left_hip_medial_contact", leftThigh,
               SimTK::Vec3(0, 0.17, 0.02), transform, params);
    addContact(model, "left_hip_lateral_contact", leftThigh,
               SimTK::Vec3(0, 0.17, -0.02), transform, params);

    // Right hip contact points
    addContact(model, "right_hip_front_contact", rightThigh,
               SimTK::Vec3(0, 0.15, 0), transform, params);
    addContact(model, "right_hip_back_contact", rightThigh,
               SimTK::Vec3(0, 0.19, 0), transform, params);
    addContact(model, "right_hip_medial_contact", rightThigh,
               SimTK::Vec3(0, 0.17, -0.02), transform, params);
    addContact(model, "right_hip_lateral_contact", rightThigh,
               SimTK::Vec3(0, 0.17, 0.02), transform, params);

    // Left knee contact points
    addContact(model, "left_knee_front_contact", leftShank,
               SimTK::Vec3(0, 0.15, 0), transform, params);
    addContact(model, "left_knee_back_contact", leftShank,
               SimTK::Vec3(0, 0.22, 0), transform, params);
    addContact(model, "left_knee_medial_contact", leftShank,
               SimTK::Vec3(0, 0.1867, 0.015), transform, params);
    addContact(model, "left_knee_lateral_contact", leftShank,
               SimTK::Vec3(0, 0.1867, -0.015), transform, params);

    // Right knee contact points
    addContact(model, "right_knee_front_contact", rightShank,
               SimTK::Vec3(0, 0.15, 0), transform, params);
    addContact(model, "right_knee_back_contact", rightShank,
               SimTK::Vec3(0, 0.22, 0), transform, params);
    addContact(model, "right_knee_medial_contact", rightShank,
               SimTK::Vec3(0, 0.1867, -0.015), transform, params);
    addContact(model, "right_knee_lateral_contact", rightShank,
               SimTK::Vec3(0, 0.1867, 0.015), transform, params);

    // Left ankle contact points
    addContact(model, "left_ankle_front_contact", leftFoot,
               SimTK::Vec3(-0.03, 0.012, 0.008), transform, params);
    addContact(model, "left_ankle_back_contact", leftFoot,
               SimTK::Vec3(-0.07, 0.012, 0.008), transform, params);
    addContact(model, "left_ankle_medial_contact", leftFoot,
               SimTK::Vec3(-0.05123, 0.012, 0.013), transform, params);
    addContact(model, "left_ankle_lateral_contact", leftFoot,
               SimTK::Vec3(-0.05123, 0.012, 0.003), transform, params);

    // Right ankle contact points
    addContact(model, "right_ankle_front_contact", rightFoot,
               SimTK::Vec3(-0.03, 0.012, -0.008), transform, params);
    addContact(model, "right_ankle_back_contact", rightFoot,
               SimTK::Vec3(-0.07, 0.012, -0.008), transform, params);
    addContact(model, "right_ankle_medial_contact", rightFoot,
               SimTK::Vec3(-0.05123, 0.012, -0.013), transform, params);
    addContact(model, "right_ankle_lateral_contact", rightFoot,
               SimTK::Vec3(-0.05123, 0.012, -0.003), transform, params);

    // Construct system
    // ---------------
    model.finalizeConnections();
    model.initSystem();
    model.updComponent<Coordinate>(
            "/jointset/pelvis_ground/pelvis_ground_coord_4")
                    .set_default_value(1.05);
    model.updComponent<Coordinate>(
            "/jointset/pelvis_ground/pelvis_ground_coord_4")
                    .set_default_speed_value(0.0);
    model.initSystem();

    // Save the model to file.
    model.print(args["<output>"].asString());

    return EXIT_SUCCESS;
}


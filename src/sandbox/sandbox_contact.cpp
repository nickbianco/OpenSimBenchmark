#include <Simbody.h>

using namespace SimTK;

// Contact parameters.
const Real fstatic = 0.9;
const Real fdynamic = 0.6;
const Real fviscous = 0.; 
const Real dissipation = 1.0;
// Converted from Scone stiffness k = 11000 N/m with radius 0.03 m.
const Real stiffness = 1e7; 
const Real tvel = 0.1;

// Body parameters.
const Real hx = 0.5;
const Real hy = 0.1;
const Real hz = 0.1;
const Vec3 halfLengths(hx, hy, hz);
const Real radius = 0.1;

// Integrator parameters.
const Real accuracy = 1e-3;
const Real finalTime = 2.0;

double testCompliantContactSubsystem(double mass, bool visualize = false) {
    
    // Define the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -YAxis, 9.8);

    ContactTrackerSubsystem  tracker(system);
    CompliantContactSubsystem contactForces(system, tracker);
    // contactForces.setTrackDissipatedEnergy(true);
    contactForces.setTransitionVelocity(tvel);

    // Contact cliques.
    ContactCliqueId clique1 = ContactSurface::createNewContactClique();
    ContactCliqueId clique2 = ContactSurface::createNewContactClique();

    // Root body.
    Inertia Iellip = mass * Inertia::ellipsoid(halfLengths);
    Body::Rigid rootBody(MassProperties(mass, Vec3(0), Iellip));
    rootBody.addDecoration(Transform(), DecorativeEllipsoid(halfLengths));

    rootBody.addDecoration(Transform(Vec3(-hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    rootBody.addContactSurface(Transform(Vec3(-hx, 0, 0)),
        ContactSurface(ContactGeometry::Sphere(radius),
                    ContactMaterial(stiffness, dissipation, fstatic, 
                                    fdynamic, fviscous)).joinClique(clique1));

    rootBody.addDecoration(Transform(Vec3(hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    rootBody.addContactSurface(Transform(Vec3(hx, 0, 0)),
        ContactSurface(ContactGeometry::Sphere(radius),
                    ContactMaterial(stiffness, dissipation, fstatic, 
                                    fdynamic, fviscous)).joinClique(clique1));
    
    MobilizedBody::Free root(matter.Ground(), Transform(Vec3(0)),
            rootBody, Transform(Vec3(0)));

    // Pendulum body.
    Body::Rigid pendulumBody(MassProperties(mass, Vec3(0), Iellip));
    pendulumBody.addDecoration(Transform(), DecorativeEllipsoid(halfLengths));
    pendulumBody.addDecoration(Transform(Vec3(hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    pendulumBody.addContactSurface(Transform(Vec3(hx, 0, 0)),
        ContactSurface(ContactGeometry::Sphere(radius),
                    ContactMaterial(stiffness, dissipation, fstatic, 
                                    fdynamic, fviscous)).joinClique(clique1));
    MobilizedBody::Ball pendulum(root, Transform(Vec3(hx, 0, 0)),
            pendulumBody, Transform(Vec3(-hx, 0, 0)));

    // Ground body.
    const Rotation R_xdown(-Pi/2,ZAxis);
    matter.Ground().updBody().addContactSurface(
        Transform(R_xdown, Vec3(0,0,0)),
        ContactSurface(ContactGeometry::HalfSpace(),
                       ContactMaterial(stiffness, dissipation, fstatic, 
                                       fdynamic, fviscous)).joinClique(clique2));

    // Visualization.
    if (visualize) {
        Visualizer viz(system);
        system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
    }

    // Initialize the system and state.
    State state = system.realizeTopology();
    root.setOneQ(state, 5, 2.0);
    root.setOneU(state, 0, 2.0);
    root.setOneU(state, 1, 2.0);
    root.setOneU(state, 2, 2.0);
    pendulum.setOneU(state, 0, 1.0);
    pendulum.setOneU(state, 1, 1.0);
    pendulum.setOneU(state, 2, 1.0);

    // Simulate for 20 seconds.
    RungeKuttaMersonIntegrator integ(system);
    integ.setAccuracy(accuracy);
    TimeStepper ts(system, integ);
    ts.initialize(state);

    const double startCPU = cpuTime(), startTime = realTime();
    ts.stepTo(finalTime);
    double realTimeElapsed = realTime() - startTime;
    double cpuTimeElapsed = cpuTime() - startCPU;
    double realTimeFactor = finalTime / realTimeElapsed;
    std::cout << "DONE: Simulated " << integ.getTime() << " seconds in "
              << realTimeElapsed << " elapsed s, CPU=" << cpuTimeElapsed << "s\n";
    std::cout << "Integrator evaluations: " << integ.getNumRealizations() << "\n";
    std::cout << "Real time factor: " << realTimeFactor << "\n";

    return realTimeFactor;
}


double testHuntCrossleyForce(double mass, bool visualize = false) {

    // MULTIBODY SYSTEM
    // ----------------
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    GeneralContactSubsystem contacts(system);
    Force::Gravity gravity(forces, matter, -YAxis, 9.8);

    // Root body.
    Inertia Iellip = mass * Inertia::ellipsoid(halfLengths);
    Body::Rigid rootBody(MassProperties(mass, Vec3(0), Iellip));
    rootBody.addDecoration(Transform(), DecorativeEllipsoid(halfLengths));
    rootBody.addDecoration(Transform(Vec3(-hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    rootBody.addDecoration(Transform(Vec3(hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    MobilizedBody::Free root(matter.Ground(), Transform(Vec3(0)),
        rootBody, Transform(Vec3(0)));

    // Pendulum body.
    Body::Rigid pendulumBody(MassProperties(mass, Vec3(0), Iellip));
    pendulumBody.addDecoration(Transform(), DecorativeEllipsoid(halfLengths));
    pendulumBody.addDecoration(Transform(Vec3(hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    MobilizedBody::Ball pendulum(root, Transform(Vec3(hx, 0, 0)),
        pendulumBody, Transform(Vec3(-hx, 0, 0)));

    // CONTACTS
    // --------
    ContactSetIndex setIndex = contacts.createContactSet();
    contacts.addBody(setIndex, root, ContactGeometry::Sphere(radius), 
                     Transform(Vec3(-hx, 0, 0)));
    contacts.addBody(setIndex, root, ContactGeometry::Sphere(radius), 
                     Transform(Vec3(hx, 0, 0)));
    contacts.addBody(setIndex, pendulum, ContactGeometry::Sphere(radius), 
                     Transform(Vec3(hx, 0, 0)));
    contacts.addBody(setIndex, matter.updGround(), ContactGeometry::HalfSpace(), 
                     Transform(Rotation(-0.5*Pi, ZAxis), Vec3(0)));
    HuntCrossleyForce hc(forces, contacts, setIndex);
    hc.setBodyParameters(ContactSurfaceIndex(0), stiffness, dissipation, 
            fstatic, fdynamic, fviscous);
    hc.setBodyParameters(ContactSurfaceIndex(1), stiffness, dissipation, 
            fstatic, fdynamic, fviscous);
    hc.setBodyParameters(ContactSurfaceIndex(2), stiffness, dissipation, 
            fstatic, fdynamic, fviscous);
    hc.setBodyParameters(ContactSurfaceIndex(3), stiffness, dissipation, 
            fstatic, fdynamic, fviscous);
    hc.setTransitionVelocity(tvel);

    // Visualization.
    if (visualize) {
        Visualizer viz(system);
        system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
    }

    // Initialize the system and state.
    State state = system.realizeTopology();
    root.setOneQ(state, 5, 2.0);
    root.setOneU(state, 0, 2.0);
    root.setOneU(state, 1, 2.0);
    root.setOneU(state, 2, 2.0);
    pendulum.setOneU(state, 0, 1.0);
    pendulum.setOneU(state, 1, 1.0);
    pendulum.setOneU(state, 2, 1.0);

    // Simulate for 20 seconds.
    RungeKuttaMersonIntegrator integ(system);
    integ.setAccuracy(accuracy);
    TimeStepper ts(system, integ);
    ts.initialize(state);

    const double startCPU = cpuTime(), startTime = realTime();
    ts.stepTo(finalTime);
    double realTimeElapsed = realTime() - startTime;
    double cpuTimeElapsed = cpuTime() - startCPU;
    double realTimeFactor = finalTime / realTimeElapsed;
    std::cout << "DONE: Simulated " << integ.getTime() << " seconds in "
              << realTimeElapsed << " elapsed s, CPU=" << cpuTimeElapsed << "s\n";
    std::cout << "Integrator evaluations: " << integ.getNumRealizations() << "\n";
    std::cout << "Real time factor: " << realTimeFactor << "\n";

    return realTimeFactor;
}

double testSmoothSphereHalfSpaceForce(double mass, bool visualize = false) {

    // MULTIBODY SYSTEM
    // ----------------
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -YAxis, 9.8);

    // Root body.
    Inertia Iellip = mass * Inertia::ellipsoid(halfLengths);
    Body::Rigid rootBody(MassProperties(mass, Vec3(0), Iellip));
    rootBody.addDecoration(Transform(), DecorativeEllipsoid(halfLengths));
    rootBody.addDecoration(Transform(Vec3(-hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    rootBody.addDecoration(Transform(Vec3(hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    MobilizedBody::Free root(matter.Ground(), Transform(Vec3(0)),
        rootBody, Transform(Vec3(0)));

    // Pendulum body.
    Body::Rigid pendulumBody(MassProperties(mass, Vec3(0), Iellip));
    pendulumBody.addDecoration(Transform(), DecorativeEllipsoid(halfLengths));
    pendulumBody.addDecoration(Transform(Vec3(hx, 0, 0)), 
            DecorativeSphere(radius).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    MobilizedBody::Ball pendulum(root, Transform(Vec3(hx, 0, 0)),
        pendulumBody, Transform(Vec3(-hx, 0, 0)));

    // CONTACTS
    // --------
    const Real cf = 1e-5;
    const Real bd = 300;
    const Real bv = 50;
    Transform groundFrame(Rotation(-0.5*Pi, ZAxis), Vec3(0));

    SmoothSphereHalfSpaceForce hc_smooth1(forces);
    hc_smooth1.setParameters(stiffness, dissipation,
        fstatic, fdynamic, fviscous, tvel, cf, bd, bv);
    hc_smooth1.setContactSphereBody(root);
    hc_smooth1.setContactSphereLocationInBody(Vec3(-hx, 0, 0));
    hc_smooth1.setContactSphereRadius(radius);
    hc_smooth1.setContactHalfSpaceFrame(groundFrame);
    hc_smooth1.setContactHalfSpaceBody(matter.Ground());

    SmoothSphereHalfSpaceForce hc_smooth2(forces);
    hc_smooth2.setParameters(stiffness, dissipation,
        fstatic, fdynamic, fviscous, tvel, cf, bd, bv);
    hc_smooth2.setContactSphereBody(root);
    hc_smooth2.setContactSphereLocationInBody(Vec3(hx, 0, 0));
    hc_smooth2.setContactSphereRadius(radius);
    hc_smooth2.setContactHalfSpaceFrame(groundFrame);
    hc_smooth2.setContactHalfSpaceBody(matter.Ground());

    SmoothSphereHalfSpaceForce hc_smooth3(forces);
    hc_smooth3.setParameters(stiffness, dissipation,
        fstatic, fdynamic, fviscous, tvel, cf, bd, bv);
    hc_smooth3.setContactSphereBody(pendulum);
    hc_smooth3.setContactSphereLocationInBody(Vec3(hx, 0, 0));
    hc_smooth3.setContactSphereRadius(radius);
    hc_smooth3.setContactHalfSpaceFrame(groundFrame);
    hc_smooth3.setContactHalfSpaceBody(matter.Ground());

    // Visualization.
    if (visualize) {
        Visualizer viz(system);
        system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
    }

    // Initialize the system and state.
    State state = system.realizeTopology();
    root.setOneQ(state, 5, 2.0);
    root.setOneU(state, 0, 2.0);
    root.setOneU(state, 1, 2.0);
    root.setOneU(state, 2, 2.0);
    pendulum.setOneU(state, 0, 1.0);
    pendulum.setOneU(state, 1, 1.0);
    pendulum.setOneU(state, 2, 1.0);

    // Simulate for 20 seconds.
    RungeKuttaMersonIntegrator integ(system);
    integ.setAccuracy(accuracy);
    TimeStepper ts(system, integ);
    ts.initialize(state);

    const double startCPU = cpuTime(), startTime = realTime();
    ts.stepTo(finalTime);
    double realTimeElapsed = realTime() - startTime;
    double cpuTimeElapsed = cpuTime() - startCPU;
    double realTimeFactor = finalTime / realTimeElapsed;
    std::cout << "DONE: Simulated " << integ.getTime() << " seconds in "
              << realTimeElapsed << " elapsed s, CPU=" << cpuTimeElapsed << "s\n";
    std::cout << "Integrator evaluations: " << integ.getNumRealizations() << "\n";
    std::cout << "Real time factor: " << realTimeFactor << "\n";

    return realTimeFactor;
}

double testExponentialSpringForce(double mass, bool visualize = false) {

    // MULTIBODY SYSTEM
    // ----------------
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -YAxis, 9.8);

    // Root body.
    Inertia Iellip = mass * Inertia::ellipsoid(halfLengths);
    Body::Rigid rootBody(MassProperties(mass, Vec3(0), Iellip));
    rootBody.addDecoration(Transform(), DecorativeEllipsoid(halfLengths));
    rootBody.addDecoration(Transform(Vec3(-hx, 0, 0)), 
            DecorativeSphere(0.001).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    rootBody.addDecoration(Transform(Vec3(hx, 0, 0)), 
            DecorativeSphere(0.001).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    MobilizedBody::Free root(matter.Ground(), Transform(Vec3(0)),
        rootBody, Transform(Vec3(0)));

    // Pendulum body.
    Body::Rigid pendulumBody(MassProperties(mass, Vec3(0), Iellip));
    pendulumBody.addDecoration(Transform(), DecorativeEllipsoid(halfLengths));
    pendulumBody.addDecoration(Transform(Vec3(hx, 0, 0)), 
            DecorativeSphere(0.001).setOpacity(0.5)
            .setRepresentation(DecorativeGeometry::DrawWireframe));
    MobilizedBody::Ball pendulum(root, Transform(Vec3(hx, 0, 0)),
        pendulumBody, Transform(Vec3(-hx, 0, 0)));

    // CONTACTS
    // --------
    Transform groundFrame(Rotation(-0.5*Pi, XAxis), Vec3(0));

    ExponentialSpringParameters params;
    double scale = 1.0;
    params.setNormalViscosity(scale*dissipation);
    params.setFrictionViscosity(scale*dissipation);
    params.setInitialMuStatic(scale*fstatic);
    params.setInitialMuKinetic(scale*fdynamic);
    params.setSettleVelocity(tvel);

    ExponentialSpringForce spring1(forces, groundFrame, root, 
                                   Vec3(-hx, 0, 0), params);
    ExponentialSpringForce spring2(forces, groundFrame, root, 
                                   Vec3(hx, 0, 0), params);
    ExponentialSpringForce spring3(forces, groundFrame, pendulum, 
                                   Vec3(hx, 0, 0), params);
                        
    // Visualization.
    if (visualize) {
        Visualizer viz(system);
        system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
    }

    // Initialize the system and state.
    State state = system.realizeTopology();
    root.setOneQ(state, 5, 2.0);
    root.setOneU(state, 0, 2.0);
    root.setOneU(state, 1, 2.0);
    root.setOneU(state, 2, 2.0);
    pendulum.setOneU(state, 0, 1.0);
    pendulum.setOneU(state, 1, 1.0);
    pendulum.setOneU(state, 2, 1.0);

    // Simulate for 20 seconds.
    RungeKuttaMersonIntegrator integ(system);
    integ.setAccuracy(accuracy);
    TimeStepper ts(system, integ);
    ts.initialize(state);

    const double startCPU = cpuTime(), startTime = realTime();
    ts.stepTo(finalTime);
    double realTimeElapsed = realTime() - startTime;
    double cpuTimeElapsed = cpuTime() - startCPU;
    double realTimeFactor = finalTime / realTimeElapsed;
    std::cout << "DONE: Simulated " << integ.getTime() << " seconds in "
              << realTimeElapsed << " elapsed s, CPU=" << cpuTimeElapsed << "s\n";
    std::cout << "Integrator evaluations: " << integ.getNumRealizations() << "\n";
    std::cout << "Real time factor: " << realTimeFactor << "\n";

    return realTimeFactor;
}


int main() {
    // Disable logging

    // testCompliantContactSubsystem(1.0, true);
    // testHuntCrossleyForce(1000.0, true);
    // testExponentialSpringForce(1.0, true);


    std::vector<double> masses = {0.1, 1.0, 10.0, 100.0, 1000.0};
    std::unordered_map<std::string, std::vector<double>> realTimeFactors;

    // Run the test.
    for (const auto& mass : masses) {
        std::cout << "Running simulation with mass: " << mass << std::endl;
        realTimeFactors["CompliantContactSubsystem"].push_back(
            testCompliantContactSubsystem(mass));
        realTimeFactors["HuntCrossleyForce"].push_back(
            testHuntCrossleyForce(mass));
        realTimeFactors["SmoothSphereHalfSpaceForce"].push_back(
            testSmoothSphereHalfSpaceForce(mass));
        realTimeFactors["ExponentialSpringForce"].push_back(
            testExponentialSpringForce(mass));
    }

    // Print a table of results.
    std::cout << "\nResults:\n";
    std::cout << "Mass\tCompliantContact\tHuntCrossley\tSmoothSphere\tExponentialSpring\n";
    for (size_t i = 0; i < masses.size(); ++i) {
        std::cout << masses[i] << "\t" 
                  << realTimeFactors["CompliantContactSubsystem"][i] << "\t" << "\t" << "\t"
                  << realTimeFactors["HuntCrossleyForce"][i] << "\t" << "\t" << "\t"
                  << realTimeFactors["SmoothSphereHalfSpaceForce"][i] << "\t" << "\t" << "\t"
                  << realTimeFactors["ExponentialSpringForce"][i] << "\n";
    }
    std::cout << "\n";

    return 0;
}
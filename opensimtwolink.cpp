
#include "opensimtwolink.h"

#include <OpenSim/OpenSim.h>

using namespace SimTK;
using namespace OpenSim; using OpenSim::Body;

Model* model;
State* state;

TwoLink::TwoLink() {

    model = new Model();
    // model->setUseVisualizer(true);

    // Two links, with mass of 1 kg, center of mass at the
    // origin of the body's frame, and moments/products of inertia of zero.
    OpenSim::Body& ground = model->getGroundBody();
    OpenSim::Body* link1 = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
    OpenSim::Body* link2 = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));

    // Joints that connect the bodies together.
    PinJoint* joint1 = new PinJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model->getGroundBody(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *link1, Vec3(-1, 0, 0), Vec3(0));
    PinJoint* joint2 = new PinJoint("elbow",
            *link1, Vec3(0), Vec3(0), *link2, Vec3(-1, 0, 0), Vec3(0));
    /*
    SliderJoint* joint1 = new SliderJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model->getGroundBody(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *link1, Vec3(0, 0, 0), Vec3(0));
            */

    //Millard2012EquilibriumMuscle* muscle1 = new
    //    Millard2012EquilibriumMuscle("muscle1", 200, 0.6, 0.55, 0);
    //muscle1->setMuscleConfiguration(true, true, 0.000);
    PathActuator* muscle1L = new PathActuator();
    muscle1L->setName("muscle1L");
    PathActuator* muscle1R = new PathActuator();
    muscle1R->setName("muscle1R");
    //muscle1->addNewPathPoint("point1", ground, Vec3(-0.1, 0.0, 0));
    ////muscle1->addNewPathPoint("point2", *link1, Vec3(-1.0, 0.1, 0));
    //muscle1->addNewPathPoint("point3", *link1, Vec3(-0.3, 0.0, 0));

    /*
    // A controller that specifies the excitation of the biceps muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*muscle1);
    // Muscle excitation is 0.3 for the first 0.5 seconds, and 1.0 thereafter.
    brain->prescribeControlForActuator("muscle1",
            new Constant(10000)); // new StepFunction(0.5, 3, 0.3, 1));
    */

    CoordinateActuator* shoulderAct = new CoordinateActuator("shoulder_coord_0");
    CoordinateActuator* elbowAct = new CoordinateActuator("elbow_coord_0");

    // Add bodies and joints to the model.
    model->addBody(link1);
    model->addBody(link2);
    model->addForce(muscle1L);
    model->addForce(muscle1R);
    //model->addForce(shoulderAct);
    model->addForce(elbowAct);
    //model->addController(brain);

    state = &model->initSystem();

    /*
    muscle1->addNewPathPoint("point1", ground, Vec3(-0.1, 0.1, 0));
    muscle1->addNewPathPoint("point2", *link1, Vec3(-0.3, 0.0, 0));
    ConditionalPathPoint* pp1 = new ConditionalPathPoint();
    pp1->setBody(ground);
    pp1->setLocation(*state, Vec3(-0.1, 0.1, 0));
    pp1->setCoordinate(*state, model->updCoordinateSet()[0]);
    pp1->setRangeMin(*state, 
    muscle1->updGeometryPath().adoptAndAppend(pp1);
    muscle1->addNewPathPoint("point2", *link1, Vec3(-0.3, 0.0, 0));
    */
    MovingPathPoint* pp1L = new MovingPathPoint();
    pp1L->setName("point1");
    pp1L->setBody(ground);
    //pp1L->setLocation(*state, Vec3(-0.1, 0.1, 0));
    pp1L->setXCoordinate(*state, model->updCoordinateSet()[0]);
    Sine* xfunc = new Sine(0.1, 1, 0.5 * Pi + 0.5 * Pi); // Constant(-0.1);
    pp1L->setXFunction(*state, *xfunc);
    pp1L->setYCoordinate(*state, model->updCoordinateSet()[0]);
    Sine* yfunc = new Sine(0.1, 1, 0.5 * SimTK::Pi);
    pp1L->setYFunction(*state, *yfunc);
    pp1L->setZCoordinate(*state, model->updCoordinateSet()[0]);
    Constant* zfunc = new Constant(0.0);
    pp1L->setZFunction(*state, *zfunc);
    muscle1L->updGeometryPath().updPathPointSet().adoptAndAppend(pp1L);
    muscle1L->addNewPathPoint("point2", *link1, Vec3(-0.3, 0.0, 0));

    MovingPathPoint* pp1R = new MovingPathPoint();
    pp1R->setName("point1");
    pp1R->setBody(ground);
    pp1R->setXCoordinate(*state, model->updCoordinateSet()[0]);
    Sine* xfunc1R = new Sine(-0.1, 1, 0.5 * Pi + 0.5 * Pi);
    pp1R->setXFunction(*state, *xfunc1R);
    pp1R->setYCoordinate(*state, model->updCoordinateSet()[0]);
    Sine* yfunc1R = new Sine(-0.1, 1, 0.5 * SimTK::Pi);
    pp1R->setYFunction(*state, *yfunc1R);
    pp1R->setZCoordinate(*state, model->updCoordinateSet()[0]);
    Constant* zfunc1R = new Constant(0.0);
    pp1R->setZFunction(*state, *zfunc1R);
    muscle1R->updGeometryPath().updPathPointSet().adoptAndAppend(pp1R);
    muscle1R->addNewPathPoint("point2", *link1, Vec3(-0.3, 0.0, 0));

    model->print("mytwolink.osim");

    state = &model->initSystem();

    /*
    model->updMatterSubsystem().setShowDefaultGeometry(true);

    RungeKuttaMersonIntegrator integrator(model->getMultibodySystem());
    Manager manager(*model, integrator);
    manager.setInitialTime(0); manager.setFinalTime(10.0);
    manager.integrate(*state);
    */

}

void TwoLink::dae(double* derivatives, double* path, double* states, 
        double* controls, double* parameters, double& time, 
        double* xad, int iphase) {

    State s = *state;
    model->getMultibodySystem().realize(s, SimTK::Stage::Position);
    model->setControls(s, SimTK::Vector(3, controls));
    model->getCoordinateSet()[0].setValue(s, states[0]);
    model->getCoordinateSet()[1].setValue(s, states[1]);
    model->getCoordinateSet()[0].setSpeedValue(s, states[2]);
    model->getCoordinateSet()[1].setSpeedValue(s, states[3]);
    model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    derivatives[0] = s.getYDot()[0];
    derivatives[1] = s.getYDot()[1];
    derivatives[2] = s.getYDot()[2];
    derivatives[3] = s.getYDot()[3];
    /*
    derivatives[0] = states[1];
    derivatives[1] = controls[0];
    */
    /*
     s = *state;
    model->getMultibodySystem().realize(s, SimTK::Stage::Position);
    model->setControls(s, SimTK::Vector(1, controls[0]));
    model->getCoordinateSet()[0].setValue(s, states[0]);
    model->getCoordinateSet()[0].setSpeedValue(s, states[1]);
    model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    derivatives[0] = s.getYDot()[0];
    derivatives[1] = s.getYDot()[1];
    */
    /*
    Model m(*model);
    State s = m.initSystem();
    m.getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    std::cout << "DEBUG HELLO" << std::endl;
    m.setControls(s, SimTK::Vector(1, controls[0]));
    std::cout << "DEBUG HELLO 1" << std::endl;
    m.getCoordinateSet()[0].setValue(s, states[0]);
    std::cout << "DEBUG HELLO 2" << std::endl;
    m.getCoordinateSet()[0].setSpeedValue(s, states[1]);
    std::cout << "DEBUG HELLO 3" << std::endl;
    m.getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    derivatives[0] = s.getYDot()[0];
    derivatives[1] = s.getYDot()[1];
    */
    /*
    model->setControls(*state, SimTK::Vector(2, controls));
    model->getCoordinateSet()[0].setValue(*state, states[0]);
    model->getCoordinateSet()[1].setValue(*state, states[1]);
    model->getCoordinateSet()[0].setSpeedValue(*state, states[2]);
    model->getCoordinateSet()[1].setSpeedValue(*state, states[3]);
    model->getMultibodySystem().realize(*state, SimTK::Stage::Acceleration);
    derivatives[0] = state->getYDot()[0];
    derivatives[1] = state->getYDot()[1];
    derivatives[2] = state->getYDot()[2];
    derivatives[3] = state->getYDot()[3];
    */

    /*
    std::cout << "y: " << state->getY() << std::endl;
    std::cout << "u: " << controls[0] << " " << controls[1] << std::endl;
    std::cout << "ydot: " << state->getYDot() << std::endl;
    */
}

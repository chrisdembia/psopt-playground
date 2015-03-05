
#include "opensimtwolink.h"

#include <OpenSim/OpenSim.h>

using namespace SimTK;
using namespace OpenSim; using OpenSim::Body;

Model* model;
State* state;

#define VIZ 0

TwoLink::TwoLink() {

    model = new Model();
#if VIZ
    model->setUseVisualizer(true);
#endif

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

    //Millard2012EquilibriumMuscle* muscle1L = new
    //    Millard2012EquilibriumMuscle("muscle1L", 200, 0.6, 0.55, 0);
    //muscle1->setMuscleConfiguration(true, true, 0.000);
    PathActuator* muscle1L = new PathActuator();
    muscle1L->setName("muscle1L");
    PathActuator* muscle1R = new PathActuator();
    muscle1R->setName("muscle1R");
    PathActuator* muscle2L = new PathActuator();
    muscle2L->setName("muscle2L");
    PathActuator* muscle2R = new PathActuator();
    muscle2R->setName("muscle2R");

    /*
    // A controller that specifies the excitation of the biceps muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*muscle1L);
    // Muscle excitation is 0.3 for the first 0.5 seconds, and 1.0 thereafter.
    brain->prescribeControlForActuator("muscle1L",
            new Constant(100)); // new StepFunction(0.5, 3, 0.3, 1));
    //brain->prescribeControlForActuator("muscle1L",
    //        new StepFunction(0.5, 3, 0.3, 1));
    */

    //CoordinateActuator* shoulderAct = new CoordinateActuator("shoulder_coord_0");
    //CoordinateActuator* elbowAct = new CoordinateActuator("elbow_coord_0");

    // Add bodies and joints to the model.
    model->addBody(link1);
    model->addBody(link2);
    model->addForce(muscle1L);
    model->addForce(muscle1R);
    model->addForce(muscle2L);
    model->addForce(muscle2R);
    //model->addForce(shoulderAct);
    //model->addForce(elbowAct);
    //model->addController(brain);

    state = &model->initSystem();

    muscle1L->addNewPathPoint("point1", ground, Vec3(-0.1, -0.1, 0));
    ConditionalPathPoint* pp1L2 = new ConditionalPathPoint();
    pp1L2->setName("pp1L2");
    pp1L2->setBody(ground);
    pp1L2->setLocation(*state, Vec3(-0.1, 0.1, 0));
    pp1L2->setCoordinate(*state, model->updCoordinateSet()[0]);
    pp1L2->setRangeMin(*state, -Infinity);
    pp1L2->setRangeMax(*state, 0.5 * Pi);
    muscle1L->updGeometryPath().updPathPointSet().adoptAndAppend(pp1L2);
    ConditionalPathPoint* pp1L3 = new ConditionalPathPoint();
    pp1L3->setName("pp1L3");
    pp1L3->setBody(ground);
    pp1L3->setLocation(*state, Vec3(0.1, 0.1, 0));
    pp1L3->setCoordinate(*state, model->updCoordinateSet()[0]);
    pp1L3->setRangeMin(*state, -Infinity);
    pp1L3->setRangeMax(*state, 0.0);
    muscle1L->updGeometryPath().updPathPointSet().adoptAndAppend(pp1L3);
    ConditionalPathPoint* pp1L4 = new ConditionalPathPoint();
    pp1L4->setName("pp1L4");
    pp1L4->setBody(ground);
    pp1L4->setLocation(*state, Vec3(0.1, -0.1, 0));
    pp1L4->setCoordinate(*state, model->updCoordinateSet()[0]);
    pp1L4->setRangeMin(*state, -Infinity);
    pp1L4->setRangeMax(*state, -0.5 * Pi);
    muscle1L->updGeometryPath().updPathPointSet().adoptAndAppend(pp1L4);
    muscle1L->addNewPathPoint("point5", *link1, Vec3(-0.3, 0.1, 0));

    muscle1R->addNewPathPoint("point1", ground, Vec3(-0.1, 0.1, 0));
    ConditionalPathPoint* pp1R2 = new ConditionalPathPoint();
    pp1R2->setName("pp1R2");
    pp1R2->setBody(ground);
    pp1R2->setLocation(*state, Vec3(-0.1, -0.1, 0));
    pp1R2->setCoordinate(*state, model->updCoordinateSet()[0]);
    pp1R2->setRangeMin(*state, -0.5 * Pi);
    pp1R2->setRangeMax(*state, Infinity);
    muscle1R->updGeometryPath().updPathPointSet().adoptAndAppend(pp1R2);
    ConditionalPathPoint* pp1R3 = new ConditionalPathPoint();
    pp1R3->setName("pp1R3");
    pp1R3->setBody(ground);
    pp1R3->setLocation(*state, Vec3(0.1, -0.1, 0));
    pp1R3->setCoordinate(*state, model->updCoordinateSet()[0]);
    pp1R3->setRangeMin(*state, 0.0);
    pp1R3->setRangeMax(*state, Infinity);
    muscle1R->updGeometryPath().updPathPointSet().adoptAndAppend(pp1R3);
    ConditionalPathPoint* pp1R4 = new ConditionalPathPoint();
    pp1R4->setName("pp1R4");
    pp1R4->setBody(ground);
    pp1R4->setLocation(*state, Vec3(0.1, 0.1, 0));
    pp1R4->setCoordinate(*state, model->updCoordinateSet()[0]);
    pp1R4->setRangeMax(*state, Infinity);
    pp1R4->setRangeMin(*state, 0.5 * Pi);
    muscle1R->updGeometryPath().updPathPointSet().adoptAndAppend(pp1R4);
    muscle1R->addNewPathPoint("point5", *link1, Vec3(-0.3, -0.1, 0));

    muscle2L->addNewPathPoint("point1", *link1, Vec3(-0.1, -0.1, 0));
    ConditionalPathPoint* pp2L2 = new ConditionalPathPoint();
    pp2L2->setName("pp2L2");
    pp2L2->setBody(*link1);
    pp2L2->setLocation(*state, Vec3(-0.1, 0.1, 0));
    pp2L2->setCoordinate(*state, model->updCoordinateSet()[1]);
    pp2L2->setRangeMin(*state, -Infinity);
    pp2L2->setRangeMax(*state, 0.5 * Pi);
    muscle2L->updGeometryPath().updPathPointSet().adoptAndAppend(pp2L2);
    ConditionalPathPoint* pp2L3 = new ConditionalPathPoint();
    pp2L3->setName("pp2L3");
    pp2L3->setBody(*link1);
    pp2L3->setLocation(*state, Vec3(0.1, 0.1, 0));
    pp2L3->setCoordinate(*state, model->updCoordinateSet()[1]);
    pp2L3->setRangeMin(*state, -Infinity);
    pp2L3->setRangeMax(*state, 0.0);
    muscle2L->updGeometryPath().updPathPointSet().adoptAndAppend(pp2L3);
    ConditionalPathPoint* pp2L4 = new ConditionalPathPoint();
    pp2L4->setName("pp2L4");
    pp2L4->setBody(*link1);
    pp2L4->setLocation(*state, Vec3(0.1, -0.1, 0));
    pp2L4->setCoordinate(*state, model->updCoordinateSet()[1]);
    pp2L4->setRangeMin(*state, -Infinity);
    pp2L4->setRangeMax(*state, -0.5 * Pi);
    muscle2L->updGeometryPath().updPathPointSet().adoptAndAppend(pp2L4);
    muscle2L->addNewPathPoint("point5", *link2, Vec3(-0.3, 0.1, 0));

    muscle2R->addNewPathPoint("point1", *link1, Vec3(-0.1, 0.1, 0));
    ConditionalPathPoint* pp2R2 = new ConditionalPathPoint();
    pp2R2->setName("pp2R2");
    pp2R2->setBody(*link1);
    pp2R2->setLocation(*state, Vec3(-0.1, -0.1, 0));
    pp2R2->setCoordinate(*state, model->updCoordinateSet()[1]);
    pp2R2->setRangeMin(*state, -0.5 * Pi);
    pp2R2->setRangeMax(*state, Infinity);
    muscle2R->updGeometryPath().updPathPointSet().adoptAndAppend(pp2R2);
    ConditionalPathPoint* pp2R3 = new ConditionalPathPoint();
    pp2R3->setName("pp2R3");
    pp2R3->setBody(*link1);
    pp2R3->setLocation(*state, Vec3(0.1, -0.1, 0));
    pp2R3->setCoordinate(*state, model->updCoordinateSet()[1]);
    pp2R3->setRangeMin(*state, 0.0);
    pp2R3->setRangeMax(*state, Infinity);
    muscle2R->updGeometryPath().updPathPointSet().adoptAndAppend(pp2R3);
    ConditionalPathPoint* pp2R4 = new ConditionalPathPoint();
    pp2R4->setName("pp2R4");
    pp2R4->setBody(*link1);
    pp2R4->setLocation(*state, Vec3(0.1, 0.1, 0));
    pp2R4->setCoordinate(*state, model->updCoordinateSet()[1]);
    pp2R4->setRangeMax(*state, Infinity);
    pp2R4->setRangeMin(*state, 0.5 * Pi);
    muscle2R->updGeometryPath().updPathPointSet().adoptAndAppend(pp2R4);
    muscle2R->addNewPathPoint("point5", *link2, Vec3(-0.3, -0.1, 0));

    model->print("mytwolink.osim");

    state = &model->initSystem();
    //model->updCoordinateSet()[0].setValue(*state, -0.5 * Pi);

#if VIZ
    model->updMatterSubsystem().setShowDefaultGeometry(true);

    RungeKuttaMersonIntegrator integrator(model->getMultibodySystem());
    Manager manager(*model, integrator);
    manager.setInitialTime(0); manager.setFinalTime(10.0);
    manager.integrate(*state);
#endif

}

void TwoLink::dae(double* derivatives, double* path, double* states, 
        double* controls, double* parameters, double& time, 
        double* xad, int iphase) {

    State s = *state;
    model->getMultibodySystem().realize(s, SimTK::Stage::Position);
    model->setControls(s, SimTK::Vector(4, controls));
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

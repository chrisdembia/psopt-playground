
#include "opensimtwolink.h"

#include <OpenSim/OpenSim.h>

using namespace SimTK;
using namespace OpenSim; using OpenSim::Body;

Model* model;
State* state;

TwoLink::TwoLink() {

    model = new Model();

    // Two links, with mass of 1 kg, center of mass at the
    // origin of the body's frame, and moments/products of inertia of zero.
    OpenSim::Body* link1 = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
    //OpenSim::Body* link2 = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));

    // Joints that connect the bodies together.
    PinJoint* joint1 = new PinJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model->getGroundBody(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *link1, Vec3(-1, 0, 0), Vec3(0));
    //PinJoint* joint2 = new PinJoint("elbow",
    //        *link1, Vec3(0), Vec3(0), *link2, Vec3(-1, 0, 0), Vec3(0));

    /*
    // Add an actuator that crosses the elbow, and a joint stop.
    Millard2012EquilibriumMuscle* muscle = new
    Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    muscle->addNewPathPoint("point1", *link1, Vec3(0, 0.8, 0));
    muscle->addNewPathPoint("point2", *link2, Vec3(0, 0.7, 0));

    // A controller that specifies the excitation of the biceps muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*muscle);
    // Muscle excitation is 0.3 for the first 0.5 seconds, and 1.0 thereafter.
    brain->prescribeControlForActuator("biceps",
    new StepFunction(0.5, 3, 0.3, 1));
    */

    CoordinateActuator* shoulderAct = new CoordinateActuator("shoulder_coord_0");
    //CoordinateActuator* elbowAct = new CoordinateActuator("elbow_coord_0");

    // Add bodies and joints to the model.
    model->addBody(link1);
    //model->addBody(link2);
    // model.addForce(muscle);
    model->addForce(shoulderAct);
    //model->addForce(elbowAct);

    state = &model->initSystem();
    // model.print("mytwolink.osim");
}

void TwoLink::dae(double* derivatives, double* path, double* states, 
        double* controls, double* parameters, double& time, 
        double* xad, int iphase) {
    model->setControls(*state, SimTK::Vector(1, controls));
    model->getCoordinateSet()[0].setValue(*state, states[0]);
    model->getCoordinateSet()[0].setSpeedValue(*state, states[1]);
    model->getMultibodySystem().realize(*state, SimTK::Stage::Acceleration);
    derivatives[0] = state->getYDot()[0];
    derivatives[1] = state->getYDot()[1];
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

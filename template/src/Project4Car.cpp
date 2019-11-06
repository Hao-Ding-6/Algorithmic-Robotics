///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
//#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <math.h>
#include <cmath>
#include <vector>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
const auto carLen = 0.05;

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the car
        const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
        projection(0) = (values[0] + values[1]) / 2.0;
        projection(1) = (values[1] + values[2]) / 2.0;

    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 0.05;

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]) / carLength;

}

// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.

    // Create 4 obstacles as street
    Rectangle r1;
    r1.x = -1;
    r1.y = -0.4;
    r1.width = 0.5;
    r1.height = 0.8;

    Rectangle r2;
    r2.x = 0.5;
    r2.y = -0.4;
    r2.width = 0.5;
    r2.height = 0.8;

    Rectangle r3;
    r3.x = -1;
    r3.x = 0.8;
    r3.width = 2;
    r3.height = 0.2;

    Rectangle r4;
    r4.x = -1;
    r4.y = -1;
    r4.width = 2;
    r4.height = 0.2;

    obstacles.push_back(r1);
    obstacles.push_back(r2);
    obstacles.push_back(r3);
    obstacles.push_back(r4);

}

// DemoControlSpace from RigidBodyPlanningWithODESolverAndControls.cpp
class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};


// Check for state validity
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state, const std::vector<Rectangle> obstacles) 
{
    /// cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    /// check validity of state defined by pos & rot
    // bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles);
    double x = pos->values[0];
    double y = pos->values[1];
    double theta = rot->value;

    bool isValid = isValidSquare(x, y, theta, carLen, obstacles);
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && isValid;
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.

    // Set state space
    auto space(std::make_shared<ob::SE2StateSpace>());

    // Set Boundary
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    space->setBounds(bounds);

    // Set control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    cspace->setBounds(cbounds);

    // define a simple setup class
    //oc::SimpleSetup ss(cspace);
    auto ss(std::make_shared<oc::SimpleSetup>(cspace));

    // set state validity checking for this space
    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si, obstacles](const ob::State *state) { return isStateValid(si, state, obstacles); });

    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));

    /// create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.2);
    start->setY(-0.6);
    start->setYaw(0.0);

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.7);
    goal->setY(0.6);
    goal->setYaw(0.0);

    /// set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);

    /// we want to have a reasonable value for the propagation step size
    ss->setup();

    return ss;

}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    
    oc::SimpleSetup *ssi = ss.get();
    //std::count << ssi->getSpaceInformation() << std::endl;
    // RRT Planner
    if(choice == 1) {
        auto planner = std::make_shared<ompl::control::RRT>(ssi->getSpaceInformation());
        ssi->setPlanner(planner);
    }

    // KPIECE1 Planner
    else if(choice == 2) {
        auto planner = std::make_shared<ompl::control::KPIECE1>(ssi->getSpaceInformation());
        ssi->getStateSpace()->registerProjection("CarProjection", ompl::base::ProjectionEvaluatorPtr(new CarProjection(ssi->getStateSpace().get())));
        planner->setProjectionEvaluator("CarProjection");

        ssi->setPlanner(planner);
    }

    // RG-RRT Planner
    else {
        auto planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }

    // Try to solve the problem 
    ob::PlannerStatus solved = ssi->solve(20.0);

    if(solved) {
        std::cout << "Solution found" << std::endl;

        // Print out the solution
        ssi->getSolutionPath().asGeometric().printAsMatrix(std::cout);

        // Print out solution to file

    }
    else {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}

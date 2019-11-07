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
#include <ompl/tools/benchmark/Benchmark.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
const auto carLen = 1;
const double pi = 3.1415;
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

    void defaultCellSizes(void) 
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.15;
        cellSizes_[1] = 0.25;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the car
        /*
        const double *values = state->as<ob::CompoundStateSpace::StateType>()
                                   ->as<ob::SE2StateSpace::StateType>()->values;

        projection(0) = values[0];
        projection(1) = values[1];
        */

        const double xValue = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->getX();
        const double yValue = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->getY();

        projection(0) = xValue;
        projection(1) = yValue;


    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    // u = { angular velocity, accleration }
    // q = (x', y', theta', v') = (vcostheta, vsintheta, angular velocity, accelration)
    double theta = q[2];
    double velocity = q[3];
    //double carLength = 0.05;
    double angularV = u[0];
    double vdot = u[1];

    // Zero out qdot
    qdot.resize (q.size (), 0);

    //qdot[0] = u[0] * cos(theta);
    //qdot[1] = u[0] * sin(theta);
    //qdot[2] = u[0] * tan(u[1]) / carLength;
    qdot[0] = velocity * cos(theta);
    qdot[1] = velocity * sin(theta);
    qdot[2] = angularV;
    qdot[3] = vdot;

}

// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()
        ->as<ob::SE2StateSpace::StateType>(0)
        ->as<ob::SO2StateSpace::StateType>(1));
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.

    // Create 4 obstacles as street
    Rectangle r1;
    r1.x = -10;
    r1.y = -4;
    r1.width = 8;
    r1.height = 8;

    Rectangle r2;
    r2.x = 5;
    r2.y = -4;
    r2.width = 8;
    r2.height = 8;

    Rectangle r3;
    r3.x = -10;
    r3.y = 8;
    r3.width = 20;
    r3.height = 2;

    Rectangle r4;
    r4.x = -10;
    r4.y = -10;
    r4.width = 20;
    r4.height = 2;

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
    const auto *cs = state->as<ob::CompoundStateSpace::StateType>();
    const auto *se2state = cs->as<ob::SE2StateSpace::StateType>(0);

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
    bool validPoint = isValidPoint(x, y, obstacles);

    //std::cout << x << ":" << y << std::endl;
    //std::cout << isValid << std::endl;

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && isValid && validPoint;
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.

    // Set state space
    /*
    auto space(std::make_shared<ob::SE2StateSpace>());
    auto rSpace(std::make_shared<ob::RealVectorStateSpace>(1));
    ob::StateSpacePtr ssp = space + rSpace;
    

    auto R2Space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds R2Bounds(2);
    R2Bounds.setLow(-10);
    R2Bounds.setHigh(10);
    R2Space->setBounds(R2Bounds);

    auto SO2Space(std::make_shared<ob::SO2StateSpace>());

    auto R1(std::make_shared<ob::RealVectorStateSpace>(1));
    ob::RealVectorBounds r1bounds(1);

    // Set Boundary
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    ob::RealVectorBounds rbounds(1);
    rbounds.setLow(-1);
    rbounds.setHigh(1);
    rSpace->setBounds(rbounds);

    // Set control space
    auto cspace(std::make_shared<DemoControlSpace>(ssp));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    // bounds for steering angle
    cbounds.setLow(0,-pi/4);
    cbounds.setHigh(0,pi/4);
    // bounds for speed
    cbounds.setLow(1,-0.1);
    cbounds.setHigh(1,0.1);

    cspace->setBounds(cbounds);

    // define a simple setup class
    //oc::SimpleSetup ss(cspace);
    auto ss(std::make_shared<oc::SimpleSetup>(cspace));

    // set state validity checking for this space
    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si, obstacles](const ob::State *state) { return isStateValid(si, state, obstacles); });

    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));
    */


    ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace());
    ompl::base::StateSpacePtr rv(new ompl::base::RealVectorStateSpace(1));
    ompl::base::StateSpacePtr space = se2 + rv;
 
    /// so2 bound is set automatically
    /// set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-20);
    bounds.setHigh(20);
 
    se2->as<ob::SE2StateSpace>()->setBounds(bounds);
 
    // set the bounds for the R part
    ob::RealVectorBounds speedBounds(1);
    speedBounds.setLow(-3);
    speedBounds.setHigh(3);
    rv->as<ob::RealVectorStateSpace>()->setBounds(speedBounds);
 
    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));
 
    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -1.5);
    cbounds.setHigh(0, 1.5);
    cbounds.setLow(1, -1);
    cbounds.setHigh(1, 1);
 
    cspace->setBounds(cbounds);
 
    // define a simple setup class
    //oc::SimpleSetup ss(cspace);
    auto ss(std::make_shared<oc::SimpleSetup>(cspace));
    // set state validity checking for this space
    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    //ss.setStateValidityChecker(std::bind(isCarStateValid, std::placeholders::_1, si, obstacles));
    ss->setStateValidityChecker(
        [si, obstacles](const ob::State *state) { return isStateValid(si, state, obstacles); });

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));


    /// create a start state
    ob::ScopedState<ob::CompoundStateSpace> start(space);
    start->as<ob::SE2StateSpace::StateType>(0)->setX(-2);
    start->as<ob::SE2StateSpace::StateType>(0)->setY(-6);
    start->as<ob::SE2StateSpace::StateType>(0)->setYaw(0.0);
    start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;
    //start->setY(0.0);
    //start->setYaw(0.0);
    //start[0] = -0.2;
    //start[1] = -0.6;
    //start[2] = 0.0;
    //start[3] = 0.0;

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::CompoundStateSpace> goal(space);
    goal->as<ob::SE2StateSpace::StateType>(0)->setX(7);
    goal->as<ob::SE2StateSpace::StateType>(0)->setY(6);
    goal->as<ob::SE2StateSpace::StateType>(0)->setYaw(0.0);
    goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;
    //goal[0] = 0.7;
    //goal[1] = 0.6;
    //goal[2] = 0.0;
    //goal[3] = 0.0;

    /// set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);
    std::cout << "is good here" << std::endl;

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
        //auto planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        //ss->setPlanner(planner);
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

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the car

    /***     Benchmark Part       ***/

    ompl::tools::Benchmark::Request request(60.0, 3076.0, 20); // at least 20 run times
    ompl::tools::Benchmark b(*ss, "Car");

    // RRT
    b.addPlanner(std::make_shared<ompl::control::RRT>(ss.get()->getSpaceInformation()));

    // KPIECE1
    auto kpiece(std::make_shared<ompl::control::KPIECE1>(ss.get()->getSpaceInformation()));
    oc::SimpleSetup *ssi = ss.get();
    ssi->getStateSpace()->registerProjection("CarProjection", ompl::base::ProjectionEvaluatorPtr(new CarProjection(ssi->getStateSpace().get())));
    kpiece->setProjectionEvaluator("CarProjection");
    b.addPlanner(kpiece);

    // RG-RRT
    b.addPlanner(std::make_shared<ompl::control::RGRRT>(ss.get()->getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile();

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


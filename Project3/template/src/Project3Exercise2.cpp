///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

// Including SimpleSetup will pull in MOST of what you need to plan
#include <ompl/geometric/SimpleSetup.h>

// Except for the state space definitions...
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

// And any planners...
#include <ompl/geometric/planners/prm/PRM.h>

// Use placeholder namespace for arguments to bound functions.
using namespace std::placeholders;

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

// This is our state validity checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::base::State *state, const std::vector<Rectangle> &obstacles)
{
    // Cast the state to a real vector state
    auto r2state = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Extract x, y
    double x = r2state->values[0];
    double y = r2state->values[1];

    return isValidPoint(x, y, obstacles);
}

// This is our state validity checker for checking if our square robot is in collision
bool isValidStateSquare(const ompl::base::State *state, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // Cast the state to a compound state
    auto cstate = state->as<ompl::base::CompoundState>();

    // Extract the real vector component (x, y)
    auto r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = r2state->values[0];
    double y = r2state->values[1];

    // Extract theta (SO(2))
    auto so2State = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    double theta = so2State->value;

    return isValidSquare(x, y, theta, sideLength, obstacles);
}

void planWithSimpleSetupR2(const std::vector<Rectangle> &obstacles)
{
    // Step 1) Create the state (configuration) space for your system
    // For a robot that can translate in the plane, we can use R^2 directly
    // We also need to set bounds on R^2
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2);  // x and y have a minimum of -2
    bounds.setHigh(2);  // x and y have a maximum of 2

    // Set the bounds on R2
    r2->setBounds(bounds);

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(r2);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker.
    // Note, we are "binding" the obstacles to the state validity checker. The
    // _1 notation is from std::placeholders and indicates "the first argument"
    // to the function pointer.
    ss.setStateValidityChecker(std::bind(isValidStatePoint, _1, obstacles));

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    ompl::base::ScopedState<> start(r2);
    start[0] = -1.3;
    start[1] = -1.3;

    ompl::base::ScopedState<> goal(r2);
    goal[0] = 1.2;
    goal[1] = 1.2;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
         ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "R2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void planWithSimpleSetupSE2(const std::vector<Rectangle> &obstacles)
{
    // Step 1) Create the state (configuration) space for your system
    // In this instance, we will plan for a square of side length 0.3
    // that both translates and rotates in the plane.
    // The state space can be easily composed of simpler state spaces
    ompl::base::StateSpacePtr se2;

    // Create the R^2 component of the space
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2);  // x and y have a minimum of -2
    bounds.setHigh(2);  // x and y have a maximum of 2

    // Set the bounds on R^2
    r2->setBounds(bounds);

    // Create the SO(2) component of the state space
    auto so2 = std::make_shared<ompl::base::SO2StateSpace>();

    // Create the compound state space (R^2 X SO(2) = SE(2))
    se2 = r2 + so2;

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem using OMPL.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(se2);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker
    // Note, we are "binding" the side length, 0.3, and the obstacles to the
    // state validity checker. The _1 notation is from std::placeholders and
    // indicates "the first argument" to the function pointer.
    ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 0.3, obstacles));

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    // You can index into the components of the state easily with ScopedState
    // The indexes correspond to the order that the StateSpace components were
    // added into the StateSpace
    ompl::base::ScopedState<> start(se2);
    start[0] = -1.3;
    start[1] = -1.3;
    start[2] = 0.0;

    ompl::base::ScopedState<> goal(se2);
    goal[0] = 1.2;
    goal[1] = 1.2;
    goal[2] = 0.0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
         ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "SE2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void planPoint(const std::vector<Rectangle> & obstacles)
{
    // TODO: Use your implementation of RTP to plan for a point robot.
    planWithSimpleSetupR2(obstacles);
}

void planBox(const std::vector<Rectangle> & obstacles)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    planWithSimpleSetupSE2(obstacles);
}

void makeEnvironment1(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your first environment.
}

void makeEnvironment2(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your second environment.
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}

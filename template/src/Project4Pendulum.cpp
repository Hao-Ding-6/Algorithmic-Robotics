///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <cmath>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include "ompl/tools/benchmark/Benchmark.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
        const double *values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        projection(0) = values[0];
        projection(1) = values[1];
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[0];
    const double omega = q[1];
    const double gravity = 9.81;

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = omega;
    qdot[1] = -gravity*cos(theta)+u[0];
}

// This is a callback method invoked after numerical integration.
void PendulumnPostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
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
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    return si->satisfiesBounds(state);
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // Create state space
    auto so2Space(std::make_shared<ob::SO2StateSpace>());
    auto r1Space(std::make_shared<ob::RealVectorStateSpace>(1));

    ob::RealVectorBounds bounds(1);
    bounds.setLow(-10);
    bounds.setHigh(10);

    r1Space->setBounds(bounds);

    ob::StateSpacePtr space = so2Space + r1Space;

    // Create control space
    auto cSpace(std::make_shared<DemoControlSpace>(space));

    ob::RealVectorBounds cBounds(2);
    cBounds.setLow(-torque);
    cBounds.setHigh(torque);
    cSpace->setBounds(cBounds);

    // define a simple setup class
    //oc::SimpleSetup ss(cspace);
    auto ss(std::make_shared<oc::SimpleSetup>(cSpace));

    // set state validity checking for this space
    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si](const ob::State *state) { return isStateValid(si, state); });

    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &PendulumnPostIntegration));

    /// create a start state
    ob::ScopedState<ob::CompoundStateSpace> start(space);
    start[0] = -M_PI / 2;
    start[1] = 0;

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::CompoundStateSpace> goal(space);
    goal[0] = M_PI / 2;
    goal[1] = 0;

    /// set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);

    /// we want to have a reasonable value for the propagation step size
    ss->setup();

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.

    oc::SimpleSetup *ssi = ss.get();

    if(choice == 1) {
        auto planner = std::make_shared<ompl::control::RRT>(ssi->getSpaceInformation());
        ssi->setPlanner(planner);
    }
    else if(choice == 2) {
        auto planner = std::make_shared<ompl::control::KPIECE1>(ssi->getSpaceInformation());
        ssi->getStateSpace()->registerProjection("PendulumProjection", ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(ssi->getStateSpace().get())));
        planner->setProjectionEvaluator("PendulumProjection");

        ssi->setPlanner(planner);
    }
    else {
        auto planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }

    // Try to solve the problem
    ob::PlannerStatus solved = ssi->solve(20.0);

    if(solved) {
        std::cout << "Solution found" << std::endl;

        // Print out the solution

        // std::cout << "every thing ok until here!!!" << std::endl;
        ssi->getSolutionPath().asGeometric().printAsMatrix(std::cout);

        // Print out solution to file

    }
    else {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the pendulum

    /***     Benchmark Part       ***/

    ompl::tools::Benchmark::Request request(60.0, 3076.0, 20); // at least 20 run times
    ompl::tools::Benchmark b(*ss, "Pendulum");

    // RRT
    b.addPlanner(std::make_shared<ompl::control::RRT>(ss.get()->getSpaceInformation()));

    // KPIECE1
    auto kpiece(std::make_shared<ompl::control::KPIECE1>(ss.get()->getSpaceInformation()));
    oc::SimpleSetup *ssi = ss.get();
    ssi->getStateSpace()->registerProjection("PendulumProjection", ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(ssi->getStateSpace().get())));
    kpiece->setProjectionEvaluator("PendulumProjection");
    b.addPlanner(kpiece);

    // RG-RRT
    b.addPlanner(std::make_shared<ompl::control::RGRRT>(ss.get()->getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
        // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}

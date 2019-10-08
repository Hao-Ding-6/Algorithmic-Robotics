///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Hao Ding (hd25) & Haoran Liang (hl74)
//////////////////////////////////////

#include "RTP.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

// TODO: Implement RTP as described

ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si) : base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");

    /**
     * Save intermediate state, but we don't use this in our RTP implementation
     */
//    Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates, "0,1");
//    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::RTP::~RTP()
{
    freeMemory();
}

void ompl::geometric::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();

    /**
     * We don't need the nn here, instead, we clear the existing motions in the vector
     */
//    if (nn_)
//        nn_->clear();
    if (!existing_motions.empty()) {
        existing_motions.clear();
    }

    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RTP::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    /**
     * nn_: A nearest-neighbors data structure containing the tree of motions
     * But we don't need to set the nn here
     */
//    if (!nn_)
//        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
//    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::RTP::freeMemory()
{
    /** use existing motions instead of nn to check whether we need to clear the memory **/
//    if (nn_)
    if (!existing_motions.empty())
    {
//        std::vector<Motion *> motions;
//        nn_->list(motions);
        for (auto &motion : existing_motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);

        si_->copyState(motion->state, st);

        /** We use vector to store existing motions instead of nn **/
//        nn_->add(motion);
        existing_motions.push_back(motion);
    }

    /** use existing motions instead of nn to check whether we need to clear the memory **/
//    if (nn_->size() == 0)
    if (existing_motions.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in data structure", getName().c_str(), existing_motions.size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState(); /** Allocate memory for a state */


    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /** We randomly choose a existing motion instead of choosing the nearest motion **/
        Motion *random_motion = existing_motions[rand() % existing_motions.size()];

        /* find closest state in the tree */
//        Motion *nmotion = nn_->nearest(rmotion); /** nmotion = nearest motion **/

        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(random_motion->state, rstate); /** rstate = resource state? **/

        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(random_motion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(random_motion->state, dstate))
        {
            /**
             * You should not implement a variant of RTP that saves intermediate states.
             */
//            if (addIntermediateStates_)
//            {
//                std::vector<base::State *> states;
//                const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);
//
//                if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
//                    si_->freeState(states[0]);
//
//                for (std::size_t i = 1; i < states.size(); ++i)
//                {
//                    Motion *motion = new Motion;
//                    motion->state = states[i];
//                    motion->parent = nmotion;
//                    nn_->add(motion);
//
//                    nmotion = motion;
//                }
//            }

            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);

            /** replace nn with existing motions vector **/
            motion->parent = random_motion;
            existing_motions.push_back(motion);
            random_motion = motion;

//            motion->parent = nmotion;
//            nn_->add(motion);
//            nmotion = motion;

            double dist = 0.0;
            bool sat = goal->isSatisfied(random_motion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = random_motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = random_motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), existing_motions.size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    /** We don't need nn here **/
//    std::vector<Motion *> motions;
//    if (nn_)
//        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : existing_motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}


///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include "ompl/datastructures/NearestNeighborsLinear.h" // use linear nn
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/SpaceInformation.h"

namespace ompl
{
    namespace control
    {
        /** \brief Based on Rapidly-exploring Random Tree */
        class RGRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            RGRRT(const SpaceInformationPtr &si);

            ~RGRRT() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion
                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                void generateReachableSet(const SpaceInformation *si, int low, int high, int pickNum) {
                    // const SpaceInformation *siC_ = (ompl::control::SpaceInformation *) si; // ???

                    for (int i = 0; i < pickNum; i ++) {
                        // set control value
                        double interval = (high - low) / (pickNum - 1);
                        double controlValue = low + i * interval;

                        // check whether the new-generated state is reachable, if yes, add it to reachable set; else delete it
                        Motion *generatedMotion = new Motion(si);
                        Control *tempControl = si->allocControl();
                        tempControl->as<RealVectorControlSpace::ControlType>()->values[0] = controlValue;
                        
                        // use getMinControlDuration() as a small time step
                        if (si->propagateWhileValid(this->state, tempControl, si->getMinControlDuration(), generatedMotion->state)) {
                            generatedMotion->control = tempControl;
                            this->reachableSet.push_back(generatedMotion); // add to reachable set
                        } else {
                            si->freeState(generatedMotion->state); // delete the state
                        }
                    }


                    // const SpaceInformation *siControl = (ompl::control::SpaceInformation *)si;
                    // const double bakControl = control->as<RealVectorControlSpace::ControlType>()->values[0];
                    // for (size_t i = 0; i < controls.size(); ++i) {  // steps is fixed to 1
                    //     base::State* newState = siControl->allocState();
                    //     control->as<RealVectorControlSpace::ControlType>()->values[0] = controls[i];
                    //     if (siControl->propagateWhileValid(state, control, siControl->getMinControlDuration(), newState) != 0) {
                    //         reachable.push_back(newState);
                    //     } else {
                    //         siControl->freeState(newState);
                    //     }
                    // }
                    // control->as<RealVectorControlSpace::ControlType>()->values[0] = bakControl;


                    // for(int i=0; i<11; i++){
                    //     double conValue = i * interval + low;
                    //     Control *c = siC_->allocControl();
                    //     auto *rcontrol =
                    //     c->as<oc::RealVectorControlSpace::ControlType>();
                    //     rcontrol->values[0] = conValue;
                            
                    //     auto *rmotion = new Motion(siC_);
                    //     base::State *result = rmotion->state;
                    //     siC_->propagate(motion->state, rcontrol, 1, result);
                    //     rmotion->control = rcontrol;
                    //     motion -> reachableSet.push_back(rmotion);            
                    // }
                }

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief The reachable set of this motion */
                std::vector<Motion *> reachableSet;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_{false};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };
    }
}

#endif

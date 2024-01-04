#include "subgoal_generator/agent.h"

namespace SubgoalGenerator
{
    Agent::Agent(const Agent &_agent)
    {
        name_ = _agent.name_;

        pose_ = _agent.pose_;
        goal_ = _agent.goal_;
        velocity_ = _agent.velocity_;
        radius_ = _agent.radius_;

        neighbors_ = _agent.neighbors_;
        VOCones_ = _agent.VOCones_;

        timeHorizon_ = _agent.timeHorizon_;
    }

    void Agent::init()
    {
        pose_.setZero();
        velocity_.setZero();
        radius_ = 0.0;

        Neighbors empty_neighbors;
        std::vector<Cone> empty_VOCones;

        neighbors_.swap(empty_neighbors);
        VOCones_.swap(empty_VOCones);

        timeHorizon_ = 0.05;
    }
} // namespace SubgoalGenerator
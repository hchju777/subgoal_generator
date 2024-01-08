#include "subgoal_generator/velocity_obstacle.h"

namespace SubgoalGenerator::VelocityObstacle
{
    Generator::Generator() {}
    Generator::~Generator()
    {
        clearAgents();
    }

    bool Generator::updateVOCones(std::string _name)
    {
        if (not(agents_.contains(_name)))
        {
            std::cerr << "VelocityObstacle::Generator::updateVOCones: "
                      << "There is no agent named " << _name << std::endl;

            return false;
        }

        Agent *agent = agents_[_name];

        for (const auto &neighborPair : agent->neighbors())
        {
            const Agent *other = neighborPair.second;

            const Eigen::Vector2d relativePosition = other->pose().position() - agent->pose().position();
            const Eigen::Vector2d relativeVelocity = agent->velocity() - other->velocity();

            const double distSq = relativePosition.squaredNorm();
            const double combinedRadius = agent->radius() + other->radius();
            const double combinedRadiusSq = combinedRadius * combinedRadius;

            Agent::Cone VOCone;
            VOCone.point_ = agent->pose().position() + other->velocity() * agent->timeHorizon();

            if (distSq > combinedRadiusSq)
            {
                // No collision
                const Eigen::Vector2d w = relativeVelocity * agent->timeHorizon() - relativePosition;
                const double wLengthSq = w.squaredNorm();
                const double dotProduct = w.dot(relativePosition);

                const double leg = std::sqrt(distSq - combinedRadiusSq);
                VOCone.radius_ = leg;
                VOCone.left_direction_ =
                    Eigen::Vector2d(
                        relativePosition.x() * leg - relativePosition.y() * combinedRadius,
                        relativePosition.x() * combinedRadius + relativePosition.y() * leg) /
                    distSq;
                VOCone.right_direction_ =
                    Eigen::Vector2d(
                        relativePosition.x() * leg + relativePosition.y() * combinedRadius,
                        -relativePosition.x() * combinedRadius + relativePosition.y() * leg) /
                    distSq;
            }
            else
            {
                // Collision
                const Eigen::Vector2d w = relativeVelocity * agent->timeHorizon() - relativePosition;
                const double wLength = w.norm();
                const Eigen::Vector2d unitW = w / wLength;

                VOCone.radius_ = std::numeric_limits<double>::infinity();
                VOCone.left_direction_ = Eigen::Vector2d(unitW.y(), unitW.x());
                VOCone.right_direction_ = Eigen::Vector2d(unitW.y(), -unitW.x());
            }

            agent->VOCones().push_back(VOCone);
        }

        return true;
    }

    void Generator::toAgent(std::string _name, const std::map<std::string, Vertex> &_vertices, Agent *_agent)
    {
        auto vertices = _vertices;

        _agent->name() = _name;
        _agent->pose() = vertices[_name].current();
        _agent->goal() = vertices[_name].goal();
    }

    void Generator::updateAgentsNeighbors(const std::map<std::string, std::list<Vertex>> &_adj_list)
    {
        auto adj_list = _adj_list;

        for (const auto &adjPair : adj_list)
        {
            if (not(agents_.contains(adjPair.first)))
                continue;

            Eigen::Vector2d position = agents_[adjPair.first]->pose().position();

            for (const auto &neighbor : adjPair.second)
            {
                Eigen::Vector2d neighbor_position = neighbor.current().position();
                Eigen::Vector2d relative_position = neighbor_position - position;

                agents_[adjPair.first]->neighbors().emplace(std::make_pair(relative_position.norm(), agents_[neighbor.name()]));
            }
        }
    }

    void Generator::reset()
    {
        clearAgents();
    }

    void Generator::clearAgents()
    {
        for (auto &agentPair : agents_)
        {
            delete agentPair.second;
            agents_.erase(agentPair.first);
        }

        std::map<std::string, Agent *> empty_agents;
        agents_.swap(empty_agents);
    }
} // namespace SubgoalGenerator::VelocityObstacle
#pragma once

#include <memory>
#include <map>
#include <iostream>

#include "subgoal_generator/agent.h"
#include "subgoal_generator/dynamic_graph_manager.h"

namespace SubgoalGenerator::VelocityObstacle
{
    class Generator
    {
    public:
        typedef std::unique_ptr<Generator> UniquePtr;
        typedef std::shared_ptr<Generator> SharedPtr;

    public:
        typedef SubgoalGenerator::DynamicGraph::Vertex Vertex;

    public:
        Generator();

        Generator(const Generator &_generator)
        {
            agents_ = _generator.agents_;
        }

        ~Generator();

    public:
        bool updateVOCones(std::string _name);

    public:
        std::map<std::string, Agent *> agents() { return agents_; }
        const std::map<std::string, Agent *> agents() const { return agents_; }

    public:
        Generator &operator=(const Generator &_rhs)
        {
            if (&_rhs != this)
                agents_ = _rhs.agents_;

            return *this;
        }

    public:
        static void toAgent(std::string _name, const std::map<std::string, Vertex> &_vertices, Agent *_agent);

        void updateAgentsNeighbors(const std::map<std::string, std::list<Vertex>> &_adj_list);

        inline void emplaceAgent(Agent *_agent)
        {
            agents_.emplace(_agent->name(), _agent);
        }

        void reset();

    protected:
        void clearAgents();

    protected:
        inline double det(const Eigen::Vector2d &_v1, const Eigen::Vector2d &_v2)
        {
            return _v1.x() * _v2.y() - _v1.y() * _v2.x();
        }

    protected:
        std::map<std::string, Agent *> agents_;

    }; // class Generator
} // namespace SubgoalGenerator::VelocityObstacle
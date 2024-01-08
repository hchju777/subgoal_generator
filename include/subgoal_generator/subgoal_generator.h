#pragma once

#include <memory>
#include <map>
#include <iostream>

#include "subgoal_generator/bvc_manager.h"
#include "subgoal_generator/dynamic_graph_manager.h"
#include "subgoal_generator/velocity_obstacle_manager.h"

namespace SubgoalGenerator
{
    class Generator
    {
    public:
        typedef std::unique_ptr<Generator> UniquePtr;
        typedef std::shared_ptr<Generator> SharedPtr;

    public:
        typedef BufferedVoronoiDiagram::VoronoiCell VoronoiCell;

    public:
        Generator();

        Generator(const Generator &_generator)
        {
            agents_ = _generator.agents_;

            graph_ = _generator.graph_;
        }

        ~Generator();

    public:
        inline Agents agents() { return agents_; }
        inline const Agents agents() const { return agents_; }

        inline DynamicGraph::Graph::SharedPtr graph() { return graph_; }
        inline const DynamicGraph::Graph::SharedPtr graph() const { return graph_; }

    public:
        void emplaceAgent(const Agent &_agent);

        void reset();

    public:
        Generator &operator=(const Generator &_rhs)
        {
            if (&_rhs != this)
            {
                agents_ = _rhs.agents_;

                graph_ = _rhs.graph_;
            }

            return *this;
        }

    public:
        bool generateBVC(
            const DynamicGraph::Vertices &_group,
            std::map<std::string, VoronoiCell> &_voronoi_diagram,
            std::map<std::string, VoronoiCell> &_buffered_voronoi_diagram);

        bool updateVOCones(const DynamicGraph::Vertices &_group);

    protected:
        Agents agents_;

        DynamicGraph::Graph::SharedPtr graph_{std::make_shared<DynamicGraph::Graph>()};

    protected:
        double communication_range_{5.0};
    }; // class Generator
} // namespace SubgoalGenerator
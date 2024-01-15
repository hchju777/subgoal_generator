#pragma once

#include <memory>
#include <map>
#include <iostream>

#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Polygon_triangulation_decomposition_2.h>

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

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

    protected:
        typedef CGAL::Partition_traits_2<Kernel> Traits;

        typedef CGAL::Quadratic_program<double> Program;
        typedef CGAL::Quadratic_program_solution<ET> Solution;

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
        bool get_BVC_Generator(
            const DynamicGraph::Vertices &_group,
            BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator);

        bool generateBVC(
            const DynamicGraph::Vertices &_group,
            BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator,
            std::map<std::string, VoronoiCell> &_voronoi_diagram,
            std::map<std::string, VoronoiCell> &_buffered_voronoi_diagram);

        bool updateVOCones(const DynamicGraph::Vertices &_group);

    protected:
        Agents agents_;

        DynamicGraph::Graph::SharedPtr graph_{std::make_shared<DynamicGraph::Graph>()};

        std::vector<BufferedVoronoiDiagram::Generator::UniquePtr> groups_;

    protected:
        double communication_range_{5.0};
    }; // class Generator
} // namespace SubgoalGenerator
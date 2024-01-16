#pragma once

#include <memory>
#include <set>
#include <stack>

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

#include "subgoal_generator/agent.h"
#include "subgoal_generator/bvc_generator.h"

#include "subgoal_generator/pibt_subgoal_util.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;

typedef Kernel::Point_2 Point_2;

namespace SubgoalGenerator::PIBT
{
    class Solver
    {
    public:
        typedef std::unique_ptr<Solver> UniquePtr;
        typedef std::shared_ptr<Solver> SharedPtr;

    protected:
        typedef CGAL::Quadratic_program<double> Program;
        typedef CGAL::Quadratic_program_solution<ET> Solution;

    public:
        typedef BufferedVoronoiDiagram::VoronoiCell VoronoiCell;

    public:
        Solver() {}

        Solver(const Agents &_agents, std::stack<std::string> _priority_graph);

        Solver(const Solver &_solver);

        ~Solver() {}

    public:
        inline Agents agents() { return agents_; }
        inline const Agents agents() const { return agents_; }

        inline BufferedVoronoiDiagram::Generator::SharedPtr &bvc_generator() { return bvc_generator_; }
        inline const BufferedVoronoiDiagram::Generator::SharedPtr &bvc_generator() const { return bvc_generator_; }

    public:
        Solver &operator=(const Solver &_rhs);

    public:
        bool solve();

    private:
        // bool priorityInheritance(const Agent &_agent, std::set<std::string>& _close, std::set<std::string> &_open);

        // bool priorityInheritance(const Agent &_child, const Agent &_parent, std::set<std::string>& _close, std::set<std::string> &_open);

        // bool priorityInheritance(const Agent &_agent, const std::vector<std::string> &_candidates, std::set<std::string>& _close, std::set<std::string> &_open);

    public:
    /**
     * @name BVC Helper function
    */
        bool get_BVC_Generator(BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator);

        bool generateBVC(
            std::map<std::string, VoronoiCell> &_voronoi_diagram,
            std::map<std::string, VoronoiCell> &_buffered_voronoi_diagram);

    protected:
        bool validate_subgoal(std::string _agentName, const Point_2 &_subgoal);

        bool is_in_the_same_face(const Point_2 &_p1, const Point_2 &_p2,
                                 const BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator);

    protected:
        Agents agents_;

        BufferedVoronoiDiagram::Generator::SharedPtr bvc_generator_;

        std::vector<std::string> priority_list_;

    }; // class Solver
} // namespace SubgoalGenerator::PIBT
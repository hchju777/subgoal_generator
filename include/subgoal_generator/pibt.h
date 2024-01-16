#pragma once

#include <memory>

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
#include "subgoal_generator/velocity_obstacle.h"

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

        Solver(const Agents &_agents);

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

    public:
    /**
     * @name BVC Helper function
    */
        bool get_BVC_Generator(BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator);

        bool generateBVC(
            std::map<std::string, VoronoiCell> &_voronoi_diagram,
            std::map<std::string, VoronoiCell> &_buffered_voronoi_diagram);

    public:
        /**
         * @name Substract VO cones and Decompose into triangular subpolygons;
         */

        bool get_truncated_polygon(
            const CGAL::Polygon_2<Kernel> &_polygon, const std::vector<Agent::Cone> _cones,
            CGAL::Polygon_2<Kernel> &_truncated_polygon);

        std::list<CGAL::Polygon_2<Kernel>> get_triangular_subPolygons(const CGAL::Polygon_2<Kernel> &_cell);

        std::list<CGAL::Polygon_2<Kernel>> get_triangular_subPolygons(const CGAL::Polygon_with_holes_2<Kernel> &_cell_w_holes);

    public:
        /**
         * @name find a subgoal using quadratic programming
         */
        bool find_subgoal(
            const Point_2 &_goal, std::list<CGAL::Polygon_2<Kernel>> &_convex_subPolygons,
            Point_2 &_subgoal);

    public:
        /**
         * @name find a garrison to inherit priority
         */
        bool find_garrison(std::string _invader, const Point_2 &_subgoal,
                           std::string &_garrison);

        bool find_garrison_point_from_voronoi_diagram(
            const Point_2 &_invader_point, const Point_2 &_subgoal,
            const BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator,
            Point_2 &_garrison_point);

    protected:
        bool find_garrison_point(
            const Point_2 &_invader_point, const Kernel::Segment_2 &_edge_seg, const VD &_vd,
            Point_2 &_garrison_point);

        std::string find_garrison_name(const Point_2 &_garrison_point);

        bool validate_subgoal(std::string _agentName, const Point_2 &_subgoal);

        bool is_in_the_same_face(const Point_2 &_p1, const Point_2 &_p2,
                                 const BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator);

    protected:
        Agents agents_;

        BufferedVoronoiDiagram::Generator::SharedPtr bvc_generator_;

    }; // class Solver
} // namespace SubgoalGenerator::PIBT
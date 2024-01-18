#pragma once

#include <CGAL/Polygon_triangulation_decomposition_2.h>

#include "subgoal_generator/bvc_generator.h"

namespace SubgoalGenerator::PIBT
{
    class SubgoalUtil
    {
    public:
        static bool find_subgoal(
            const Point_2 &_goal, const std::list<CGAL::Polygon_2<Kernel>> &_subpolygons, Point_2 &_subgoal);

        static bool find_subgoal(
            const Point_2 &_goal, const std::list<CGAL::Polygon_2<Kernel>> &_convex_subpolygons,
            Point_2 &_subgoal, double &_min_objective_value);

    protected:
        static std::list<CGAL::Polygon_2<Kernel>> triangular_decompose(const CGAL::Polygon_2<Kernel> &_cell);

        static std::list<CGAL::Polygon_2<Kernel>> triangular_decompose(const CGAL::Polygon_with_holes_2<Kernel> &_cell_w_holes);

    }; // class SubgoalUtil

} // namespace SubgoalGenerator::PIBT
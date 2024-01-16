#pragma once

// CGAL includes
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

#include "subgoal_generator/bvc_generator.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;

typedef Kernel::Point_2 Point_2;

namespace SubgoalGenerator::PIBT
{
    class SubgoalUtil
    {
    protected:
        typedef CGAL::Quadratic_program<double> Program;
        typedef CGAL::Quadratic_program_solution<ET> Solution;

    public:
        static bool find_subgoal(
            const Point_2 &_goal, std::list<CGAL::Polygon_2<Kernel>> &_convex_subPolygons,
            Point_2 &_subgoal);

    protected:
        static void set_quadratic_program(Program &_qp, const Point_2 &_goal, const CGAL::Polygon_2<Kernel> &_polygon);
        
    }; // class SubgoalUtil

} // namespace SubgoalGenerator::PIBT
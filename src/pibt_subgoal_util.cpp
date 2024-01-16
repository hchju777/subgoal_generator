#include "subgoal_generator/pibt_subgoal_util.h"

namespace SubgoalGenerator::PIBT
{
    bool SubgoalUtil::find_subgoal(
        const Point_2 &_goal, std::list<CGAL::Polygon_2<Kernel>> &_convex_subPolygons,
        Point_2 &_subgoal)
    {
        double min_objective_value = std::numeric_limits<double>::max();

        for (const auto &subPolygon : _convex_subPolygons)
        {
            Program qp(CGAL::SMALLER, false, 0, false, 0);

            SubgoalUtil::set_quadratic_program(qp, _goal, subPolygon);

            Solution solution = CGAL::solve_quadratic_program(qp, ET());

            if (solution.is_infeasible())
                continue;

            double objective_value = CGAL::to_double(solution.objective_value());
            if (objective_value < min_objective_value)
            {
                min_objective_value = objective_value;

                const auto &viter = solution.variable_values_begin();

                double x = CGAL::to_double(*viter);
                double y = CGAL::to_double(*(viter + 1));

                _subgoal = Point_2(x, y);
            }
        }

        return (min_objective_value != std::numeric_limits<double>::max());
    }

    void SubgoalUtil::set_quadratic_program(Program &_qp, const Point_2 &_goal, const CGAL::Polygon_2<Kernel> &_polygon)
    {
        const int X = 0;
        const int Y = 1;

        // Minimize: (x - goal.x)^2 + (y - goal.y)^2
        _qp.set_d(X, X, 2);
        _qp.set_d(Y, Y, 2);
        _qp.set_c(X, -2 * CGAL::to_double(_goal.x()));
        _qp.set_c(Y, -2 * CGAL::to_double(_goal.y()));
        _qp.set_c0(CGAL::to_double(_goal.x() * _goal.x() + _goal.y() * _goal.y()));

        // Add linear inequalities to represent polygon edges
        for (std::size_t i = 0; i < _polygon.size(); ++i)
        {
            const Point_2 &p1 = _polygon[i];
            const Point_2 &p2 = _polygon[(i + 1) % _polygon.size()];

            Kernel::Line_2 line(p1, p2);

            // Constraint: -ax - by <= c
            _qp.set_a(X, i, -CGAL::to_double(line.a()));
            _qp.set_a(Y, i, -CGAL::to_double(line.b()));
            _qp.set_b(i, CGAL::to_double(line.c()));
        }
    }

} // namespace SubgoalGenerator::PIBT
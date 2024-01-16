#include "subgoal_generator/pibt.h"

namespace SubgoalGenerator::PIBT
{
    Solver::Solver(const Agents &_agents)
        : agents_(_agents)
    {
        get_BVC_Generator(bvc_generator_);
    }

    Solver::Solver(const Solver &_solver)
    {
        agents_ = _solver.agents_;

        bvc_generator_ = _solver.bvc_generator_;
    }

    Solver &Solver::operator=(const Solver &_rhs)
    {
        if (&_rhs != this)
        {
            agents_ = _rhs.agents_;

            bvc_generator_ = _rhs.bvc_generator_;
        }

        return *this;
    }

    bool Solver::solve()
    {
        std::map<std::string, BufferedVoronoiDiagram::VoronoiCell> voronoi_diagram, buffered_voronoi_diagram;
        if (not(generateBVC(voronoi_diagram, buffered_voronoi_diagram)))
        {
            std::cerr << "SubgoalGenerator::Generator::solve(): "
                      << "Fail to generate voronoi diagram." << std::endl;
            return false;
        }

        return true;
    }

    bool Solver::get_BVC_Generator(BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator)
    {
        std::vector<Site_2> points;

        for (const auto &agentPair : agents_)
        {
            const Agent &agent = agentPair.second;

            points.push_back(Site_2(agent.pose().x(), agent.pose().y()));
        }

        _bvc_generator = std::make_shared<BufferedVoronoiDiagram::Generator>(points);

        return true;
    }

    bool Solver::generateBVC(
        std::map<std::string, VoronoiCell> &_voronoi_diagram,
        std::map<std::string, VoronoiCell> &_buffered_voronoi_diagram)
    {
        for (const auto &agentPair : agents_)
        {
            const Agent &agent = agentPair.second;

            VoronoiCell voronoi_cell;
            voronoi_cell.first = Point_2(agent.pose().x(), agent.pose().y());
            if (not(bvc_generator_->get_polygon(voronoi_cell.first, voronoi_cell.second)))
            {
                std::cerr << "SubgoalGenerator::Generator::generateBVC(): "
                          << "There is no voronoi cell." << std::endl;

                return false;
            }
            _voronoi_diagram.emplace(agent.name(), voronoi_cell);

            VoronoiCell buffered_voronoi_cell = voronoi_cell;
            if (not(bvc_generator_->convert_to_bvc(buffered_voronoi_cell.second, agent.radius())))
            {
                // There is no buffered voronoi cell
                continue;
            }
            _buffered_voronoi_diagram.emplace(agent.name(), buffered_voronoi_cell);
        }

        return true;
    }

    bool Solver::get_truncated_polygon(
        const CGAL::Polygon_2<Kernel> &_polygon, const std::vector<Agent::Cone> _cones,
        CGAL::Polygon_2<Kernel> &_truncated_polygon)
    {
        _truncated_polygon = _polygon;

        for (const auto &cone : _cones)
        {
            CGAL::Polygon_2<Kernel> cone_polygon;
            cone_polygon.push_back(Point_2(cone.point_.x(), cone.point_.y()));

            double right_angle = std::atan2(cone.right_direction_.y(), cone.right_direction_.x());
            double left_angle = std::atan2(cone.left_direction_.y(), cone.left_direction_.x());
            left_angle = right_angle < left_angle ? left_angle : left_angle + 2 * M_PI;

            for (double angle = right_angle - 1e-8; angle < left_angle; angle = angle + M_PI / 18)
            {
                cone_polygon.push_back(Point_2(cone.point_.x() + 1000 * std::cos(angle),
                                               cone.point_.y() + 1000 * std::sin(angle)));
            }
            cone_polygon.push_back(Point_2(cone.point_.x() + 1000 * std::cos(left_angle + 1e-8),
                                           cone.point_.y() + 1000 * std::sin(left_angle + 1e-8)));

            std::list<CGAL::Polygon_with_holes_2<Kernel>> truncated_poly_w_holes;
            CGAL::difference(_truncated_polygon, cone_polygon, std::back_inserter(truncated_poly_w_holes));

            if (truncated_poly_w_holes.size() == 1)
                _truncated_polygon = truncated_poly_w_holes.front().outer_boundary();
            else
                return false;
        }

        return true;
    }

    std::list<CGAL::Polygon_2<Kernel>> Solver::get_triangular_subPolygons(const CGAL::Polygon_2<Kernel> &_cell)
    {
        CGAL::Polygon_triangulation_decomposition_2<Kernel> triangular_decomp;

        std::list<CGAL::Polygon_2<Kernel>> triangular_decomp_poly_list;
        triangular_decomp(_cell, std::back_inserter(triangular_decomp_poly_list));

        return triangular_decomp_poly_list;
    }

    std::list<CGAL::Polygon_2<Kernel>> Solver::get_triangular_subPolygons(const CGAL::Polygon_with_holes_2<Kernel> &_cell_w_holes)
    {
        CGAL::Polygon_triangulation_decomposition_2<Kernel> triangular_decomp;

        std::list<CGAL::Polygon_2<Kernel>> triangular_decomp_poly_list;
        triangular_decomp(_cell_w_holes, std::back_inserter(triangular_decomp_poly_list));

        return triangular_decomp_poly_list;
    }

    bool Solver::find_subgoal(
        const Point_2 &_goal, std::list<CGAL::Polygon_2<Kernel>> &_convex_subPolygons,
        Point_2 &_subgoal)
    {
        double min_objective_value = std::numeric_limits<double>::max();

        for (const auto &subPolygon : _convex_subPolygons)
        {
            Program qp(CGAL::SMALLER, false, 0, false, 0);

            const int X = 0;
            const int Y = 1;

            // Minimize: (x - goal.x)^2 + (y - goal.y)^2
            qp.set_d(X, X, 2);
            qp.set_d(Y, Y, 2);
            qp.set_c(X, -2 * CGAL::to_double(_goal.x()));
            qp.set_c(Y, -2 * CGAL::to_double(_goal.y()));
            qp.set_c0(CGAL::to_double(_goal.x() * _goal.x() + _goal.y() * _goal.y()));

            // Add linear inequalities to represent polygon edges
            for (std::size_t i = 0; i < subPolygon.size(); ++i)
            {
                const Point_2 &p1 = subPolygon[i];
                const Point_2 &p2 = subPolygon[(i + 1) % subPolygon.size()];

                Kernel::Line_2 line(p1, p2);

                // Constraint: -ax - by <= c
                qp.set_a(X, i, -CGAL::to_double(line.a()));
                qp.set_a(Y, i, -CGAL::to_double(line.b()));
                qp.set_b(i, CGAL::to_double(line.c()));
            }

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

        if (min_objective_value != std::numeric_limits<double>::max())
            return true;
        else
            return false;
    }

    bool Solver::find_garrison(std::string _invader, const Point_2 &_subgoal,
                               std::string &_garrison)
    {
        if (not(agents_.contains(_invader)))
        {
            std::cerr << "PIBT::Solver::validate_subgoal: "
                      << "There is no agent named " << _invader << " or "
                      << "the gropuID is invalid" << std::endl;
            return false;
        }

        if (not(validate_subgoal(_invader, _subgoal)))
        {
            std::cerr << "PIBT::Solver::find_garrison: "
                      << "Invalid subgoal " << _subgoal << " for " << _invader << std::endl;
            return false;
        }

        const Agent &invader = agents_[_invader];

        Point_2 invader_position(invader.pose().x(), invader.pose().y()), garrison_point;

        if (not(find_garrison_point_from_voronoi_diagram(invader_position, _subgoal, bvc_generator_, garrison_point)))
            return false;

        _garrison = find_garrison_name(garrison_point);

        return true;
    }

    bool Solver::find_garrison_point_from_voronoi_diagram(
        const Point_2 &_invader_point, const Point_2 &_subgoal,
        const BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator,
        Point_2 &_garrison_point)
    {
        CGAL::Polygon_2<Kernel> vn_poly;
        if (not(_bvc_generator->get_raw_voronoi_polygon(_subgoal, vn_poly)))
            return false;

        Kernel::Direction_2 subgoal_dir = Kernel::Ray_2(_invader_point, _subgoal).direction();
        Point_2 subgoal_seg_end(_invader_point.x() + 1000 * subgoal_dir.dx(),
                                _invader_point.y() + 1000 * subgoal_dir.dy());

        Kernel::Segment_2 subgoal_seg(_invader_point, subgoal_seg_end);
        for (size_t i = 0; i < vn_poly.size(); ++i)
        {
            Kernel::Segment_2 edge_seg(vn_poly[i], vn_poly[(i + 1) % vn_poly.size()]);

            if (not(CGAL::intersection(edge_seg, subgoal_seg)))
                continue;

            return (find_garrison_point(_invader_point, edge_seg, _bvc_generator->vd(), _garrison_point));
        }

        std::cerr << "PIBT::Solver::find_garrison_point_from_voronoi_diagram: "
                  << "No edge does not intersect with a line from invader to subgoal" << std::endl;
        return false;
    }

    bool Solver::find_garrison_point(
        const Point_2 &_invader_point, const Kernel::Segment_2 &_edge_seg, const VD &_vd,
        Point_2 &_garrison_point)
    {
        Point_2 norm_line_end(_invader_point.x() + _edge_seg.direction().dy(),
                              _invader_point.y() - _edge_seg.direction().dx());
        Kernel::Line_2 norm_line(_invader_point, norm_line_end);

        const auto midpoint = CGAL::intersection(_edge_seg, norm_line);

        if (const Point_2 *p = boost::get<Point_2>(&*midpoint))
        {
            auto dx = p->x() - _invader_point.x();
            auto dy = p->y() - _invader_point.y();

            Locate_result garrison_lr = _vd.locate(Point_2(_invader_point.x() + 2 * dx, _invader_point.y() + 2 * dy));

            Face_handle *garrison_fh = boost::get<Face_handle>(&garrison_lr);

            if (garrison_fh)
            {
                _garrison_point = (*garrison_fh)->dual()->point();

                return true;
            }

            std::cerr << "PIBT::Solver::find_garrison_point: "
                      << "The garrison point is not in any voronoi cells." << std::endl;
        }
        else
        {
            std::cerr << "PIBT::Solver::find_garrison_point: "
                      << "The edge and the normal line do not intersect at a point" << std::endl;
        }

        return false;
    }

    std::string Solver::find_garrison_name(const Point_2 &_garrison_point)
    {
        double min_distSqrt = std::numeric_limits<double>::max();
        std::string garrison = std::string();

        for (const auto &agentPair : agents_)
        {
            const Agent &agent = agentPair.second;

            double dx = agent.pose().x() - CGAL::to_double(_garrison_point.x());
            double dy = agent.pose().y() - CGAL::to_double(_garrison_point.y());

            double distSqrt = dx * dx + dy * dy;
            if (distSqrt < min_distSqrt)
            {
                min_distSqrt = distSqrt;
                garrison = agent.name();
            }
        }

        return garrison;
    }

    bool Solver::validate_subgoal(std::string _agentName, const Point_2 &_subgoal)
    {
        if (not(agents_.contains(_agentName)))
        {
            std::cerr << "PIBT::Solver::validate_subgoal: "
                      << "There is no agent named " << _agentName << " or "
                      << "the groupID is invalid" << std::endl;
            return false;
        }

        const auto &agent = agents_[_agentName];

        return is_in_the_same_face(Point_2(agent.pose().x(), agent.pose().y()), _subgoal, bvc_generator_);
    }

    bool Solver::is_in_the_same_face(const Point_2 &_p1, const Point_2 &_p2,
                                     const BufferedVoronoiDiagram::Generator::SharedPtr &_bvc_generator)
    {
        Locate_result p1_lr = _bvc_generator->vd().locate(_p1);
        Locate_result p2_lr = _bvc_generator->vd().locate(_p2);

        Face_handle *p1_fh = boost::get<Face_handle>(&p1_lr);
        Face_handle *p2_fh = boost::get<Face_handle>(&p2_lr);

        return (p1_fh == p2_fh);
    }

} // namespace SubgoalGenerator::PIBT
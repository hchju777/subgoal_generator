#include "subgoal_generator/subgoal_generator.h"

namespace SubgoalGenerator
{
    Generator::Generator()
    {
        reset();

        std::cout << "Subgoal Generator has been initilized." << std::endl;
    }

    Generator::~Generator()
    {
        std::cout << "Subgoal Generator has been terminated." << std::endl;
    }

    void Generator::emplaceAgent(const Agent &_agent)
    {
        agents_.emplace(_agent.name(), _agent);

        graph_->addVertex(DynamicGraph::Vertex(_agent.name(), _agent.pose(), _agent.goal()), communication_range_);
    }

    void Generator::reset()
    {
        std::map<std::string, Agent> empty_agents;
        agents_.swap(agents_);
        agents_.clear();

        graph_->reset();
    }

    bool Generator::generateBVC(
        const DynamicGraph::Vertices &_group,
        std::map<std::string, VoronoiCell> &_voronoi_diagram,
        std::map<std::string, VoronoiCell> &_buffered_voronoi_diagram)
    {
        std::vector<Site_2> points;

        for (const auto &vertexPair : _group)
        {
            const DynamicGraph::Vertex &vertex = vertexPair.second;

            if (not(agents_.contains(vertex.name()) or graph_->vertices().contains(vertex.name())))
            {
                std::cerr << "SubgoalGenerator::Generator::generateBVC(): "
                          << "There is no agent named" << vertex.name() << std::endl;

                return false;
            }

            points.push_back(Site_2(vertex.current().x(), vertex.current().y()));
        }

        BufferedVoronoiDiagram::Generator::UniquePtr bvc_generator =
            std::make_unique<BufferedVoronoiDiagram::Generator>(points);

        for (const auto &vertexPair : _group)
        {
            const DynamicGraph::Vertex &vertex = vertexPair.second;

            VoronoiCell voronoi_cell;
            voronoi_cell.first = Point_2(vertex.current().x(), vertex.current().y());
            if (not(bvc_generator->get_polygon(voronoi_cell.first, voronoi_cell.second)))
            {
                std::cerr << "SubgoalGenerator::Generator::generateBVC(): "
                          << "There is no voronoi cell." << std::endl;

                return false;
            }
            _voronoi_diagram.emplace(vertex.name(), voronoi_cell);

            VoronoiCell buffered_voronoi_cell = voronoi_cell;
            if (not(bvc_generator->convert_to_bvc(buffered_voronoi_cell.second, agents_[vertex.name()].radius())))
            {
                // There is no buffered voronoi cell
                continue;
            }
            _buffered_voronoi_diagram.emplace(vertex.name(), buffered_voronoi_cell);
        }

        return true;
    }

    bool Generator::updateVOCones(const DynamicGraph::Vertices &_group)
    {
        VelocityObstacle::Generator::UniquePtr vo_generator = std::make_unique<VelocityObstacle::Generator>();
        for (const auto &vertexPair : _group)
        {
            Agent *agent = new Agent();
            *agent = agents_[vertexPair.first];

            vo_generator->emplaceAgent(agent);
        }

        vo_generator->updateAgentsNeighbors(graph_->adj_list());

        for (const auto &agentPair : vo_generator->agents())
        {
            std::string agentName = agentPair.first;

            vo_generator->updateVOCones(agentName);
            agents_[agentName].VOCones() = vo_generator->agents()[agentName]->VOCones();
        }

        return true;
    }

    std::list<CGAL::Polygon_2<Kernel>> Generator::get_convex_subPolygons(const CGAL::Polygon_2<Kernel> &_cell)
    {
        Traits::Polygon_2 concave_polygon;

        std::size_t idx = 0;
        for (auto viter = _cell.vertices_begin(); viter != _cell.vertices_end(); ++viter)
        {
            auto foo = Traits::Point_2(*viter);
            concave_polygon.push_back(foo);
        }

        std::list<Traits::Polygon_2> convex_subPolygons;
        CGAL::approx_convex_partition_2(concave_polygon.vertices_begin(), concave_polygon.vertices_end(),
                                        std::back_inserter(convex_subPolygons));

        std::list<CGAL::Polygon_2<Kernel>> results;
        for (const auto &poly : convex_subPolygons)
        {
            CGAL::Polygon_2<Kernel> polygon;
            for (const auto &p : poly.container())
                polygon.push_back(p);

            results.emplace_back(polygon);
        }

        return results;
    }

    bool Generator::find_subgoal(
        const Point_2 &_goal, std::list<CGAL::Polygon_2<Kernel>> &_convex_subPolygons,
        Point_2 &_subgoal)
    {
        double min_objective_function = std::numeric_limits<double>::max();

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
            if (objective_value < min_objective_function)
            {
                min_objective_function = objective_value;

                const auto &viter = solution.variable_values_begin();

                double x = CGAL::to_double(*viter);
                double y = CGAL::to_double(*(viter + 1));

                _subgoal = Point_2(x, y);
            }
        }

        if (min_objective_function != std::numeric_limits<double>::max())
            return true;
        else
            return false;
    }

} // namespace SubgoalGenerator
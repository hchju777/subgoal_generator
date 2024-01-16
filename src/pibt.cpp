#include "subgoal_generator/pibt.h"

namespace SubgoalGenerator::PIBT
{
    Solver::Solver(const Agents &_agents, std::stack<std::string> _priority_graph)
        : agents_(_agents)
    {
        priority_list_.clear();
        while (not(_priority_graph.empty()))
        {
            std::string agentName = _priority_graph.top();
            _priority_graph.pop();

            if (agents_.contains(agentName))
                priority_list_.push_back(agentName);
        }

        get_BVC_Generator(bvc_generator_);
    }

    Solver::Solver(const Solver &_solver)
    {
        agents_ = _solver.agents_;

        bvc_generator_ = _solver.bvc_generator_;

        priority_list_ = _solver.priority_list_;
    }

    Solver &Solver::operator=(const Solver &_rhs)
    {
        if (&_rhs != this)
        {
            agents_ = _rhs.agents_;

            bvc_generator_ = _rhs.bvc_generator_;

            priority_list_ = _rhs.priority_list_;
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

        std::set<std::string> open, close;
        for (const auto &agentPair : agents_)
        {
            std::string agentName = agentPair.first;

            open.emplace(agentName);
        }

        while (not(open.empty()))
        {
            Agent agent = agents_[priority_list_.front()];

            // priorityInheritance(agent, close, open);
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
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

            priorityInheritance(agent.name(), buffered_voronoi_diagram, close, open);

            priority_list_.remove_if([&open](std::string _agentName)
                                     { return not(open.contains(_agentName)); });
        }

        return true;
    }

    std::list<Solver::Candidate> Solver::createCandidates(
        Agent &_agent, const VoronoiCell &_bvc,
        const std::set<std::string> &_close, const std::set<std::string> &_open, std::string _parent)
    {
        std::set<std::string> close = _close;

        if (_parent != std::string())
            close.emplace(_parent);

        std::list<Candidate> candidates;

        std::vector<Agent::Cone> cones;

        for (auto iter = _agent.VOCones().begin(); iter != _agent.VOCones().end();)
        {
            if (_open.contains(iter->neighbor_))
            {
                iter = _agent.VOCones().erase(iter);
            }
            else
            {
                cones.push_back(*iter);
                ++iter;
            }
        }

        auto rawCandidates = CandidatesUtil::createRawCandidates(_agent, _bvc, bvc_generator_->vd());
        for (const auto rawCandidatePair : rawCandidates)
        {
            const Point_2 &neighborSite = rawCandidatePair.first;

            std::pair<std::string, Agent> neighborPair = *std::min_element(
                agents_.begin(), agents_.end(),
                [neighborSite](const auto &_agentPair1, const auto &_agentPair2)
                {
                    const double dx1 = _agentPair1.second.pose().x() - CGAL::to_double(neighborSite.x());
                    const double dy1 = _agentPair1.second.pose().y() - CGAL::to_double(neighborSite.y());

                    const double dx2 = _agentPair2.second.pose().x() - CGAL::to_double(neighborSite.x());
                    const double dy2 = _agentPair2.second.pose().y() - CGAL::to_double(neighborSite.y());

                    return (dx1 * dx1 + dy1 * dy1) < (dx2 * dx2 + dy2 * dy2);
                });

            if (neighborPair.first != _agent.name() and close.contains(neighborPair.first))
                continue;

            const CGAL::Polygon_2<Kernel> &triangular_subpolygon = rawCandidatePair.second;
            const std::list<CGAL::Polygon_2<Kernel>> &truncated_polygon = CandidatesUtil::get_truncated_polygon(triangular_subpolygon, cones);

            double truncated_area = std::accumulate(truncated_polygon.begin(), truncated_polygon.end(), 0.0,
                                                    [](double _sum, const CGAL::Polygon_2<Kernel> &_poly)
                                                    { return _sum += CGAL::to_double(_poly.area()); });

            if (truncated_area < 0.2)
                continue;

            candidates.emplace_back(neighborPair.first, truncated_polygon);
        }

        return candidates;
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

    bool Solver::priorityInheritance(
        const std::string _agentName, std::map<std::string, BufferedVoronoiDiagram::VoronoiCell> &_buffered_voronoi_diagram,
        std::set<std::string> &_close, std::set<std::string> &_open)
    {
        if (not(_open.contains(_agentName)))
            return true;

        std::list<Candidate> candidates = createCandidates(
            agents_[_agentName], _buffered_voronoi_diagram[_agentName], _close, _open);

        return priorityInheritance(_agentName, _buffered_voronoi_diagram, candidates, _close, _open);
    }

    bool Solver::priorityInheritance(
        const std::string _childName, const std::string _parentName,
        std::map<std::string, BufferedVoronoiDiagram::VoronoiCell> &_buffered_voronoi_diagram,
        std::set<std::string> &_close, std::set<std::string> &_open)
    {
        if (not(_open.contains(_childName)))
            return true;

        std::list<Candidate> candidates = createCandidates(
            agents_[_childName], _buffered_voronoi_diagram[_childName], _close, _open, _parentName);

        return priorityInheritance(_childName, _buffered_voronoi_diagram, candidates, _close, _open);
    }

    bool Solver::priorityInheritance(
        const std::string _agentName, std::map<std::string, BufferedVoronoiDiagram::VoronoiCell> &_buffered_voronoi_diagram,
        std::list<Candidate> _candidates, std::set<std::string> &_close, std::set<std::string> &_open)
    {
        Agent &agent = agents_[_agentName];

        std::cout << agent.name() << std::endl;

        _open.erase(_agentName);

        while (not(_candidates.empty()))
        {
            Candidate candidate = _candidates.front();
            std::string candidateName = candidate.first;
            _close.emplace(candidateName);

            if (_agentName != candidateName and not(priorityInheritance(candidateName, _agentName, _buffered_voronoi_diagram, _close, _open)))
            {
                _candidates.remove_if([&_close](Candidate _candidate)
                                      { return _close.contains(_candidate.first); });

                continue;
            }

            Point_2 goal(agent.goal().x(), agent.goal().y());
            Point_2 subgoal;

            if (SubgoalUtil::find_subgoal(goal, candidate.second, subgoal))
            {
                agent.subgoal().x() = CGAL::to_double(subgoal.x());
                agent.subgoal().y() = CGAL::to_double(subgoal.y());

                agent.subgoal_fixed() = true;

                return true;
            }
            _candidates.pop_front();
        }

        agent.subgoal() = agent.pose();
        agent.subgoal_fixed() = true;

        return false;
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
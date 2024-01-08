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

} // namespace SubgoalGenerator
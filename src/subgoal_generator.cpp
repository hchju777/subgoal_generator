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

        std::vector<PIBT::Solver::SharedPtr> empty_solvers;
        solvers_.swap(empty_solvers);
        solvers_.clear();

        graph_->reset();
    }

    bool Generator::generate_subgoals(Agents &_agents)
    {
        reset();

        for (const auto &agentPair : _agents)
        {
            const Agent &agent = agentPair.second;
            emplaceAgent(agent);
        }

        std::list<DynamicGraph::Vertices> groupList = graph_->generateGroupList();
        std::stack<std::string> priority_graph = graph_->topologicalSort();

        for (const auto &group : graph_->generateGroupList())
        {
            Agents agentsGroup;
            for (const auto &vertexPair : group)
            {
                std::string name = vertexPair.first;

                agentsGroup.emplace(name, agents_[name]);
            }

            PIBT::Solver::SharedPtr pibt_solver = std::make_shared<PIBT::Solver>(agentsGroup, priority_graph);
            solvers_.push_back(pibt_solver);
        }

        for (const auto &solver : solvers_)
        {
            if (solver->solve())
            {
            }
        }

        return false;
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
#include "subgoal_generator/pibt_manager.h"

namespace SubgoalGenerator::PIBT
{
    void Manager::printAgents(const Agents &_agents)
    {
        for (const auto &agentPair : _agents)
        {
            std::cout << agentPair.second << std::endl;
        }
    }

    void Manager::printGraph(const DynamicGraph::Graph::SharedPtr &_graph)
    {
        DynamicGraph::Manager::printVertices(_graph->vertices());
        DynamicGraph::Manager::printAdjList(_graph->adj_list());
    }

    void Manager::printPriority(const std::stack<std::string> &_priority_list)
    {
        DynamicGraph::Manager::printPriority(_priority_list);
    }

    void Manager::clearResultDir(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;

        if (isDirExists(_dirName))
            std::filesystem::remove_all(directory_path_string);

        std::filesystem::create_directories(directory_path_string);
    }

    bool Manager::exportDBList(
        const std::list<Manager::ExportDB> &_DBList,
        std::string _dirName, std::string _fileName)
    {
        if (not(isDirExists(_dirName)))
        {
            std::cerr << "DynamicGraph::Manager::exportDynamicGraph: "
                      << "There is no such directory named " << _dirName << std::endl;
            return false;
        }

        YAML::Node subgoal_generator_result;

        for (const auto & exportDB : _DBList)
            exportDBData(exportDB, subgoal_generator_result);

        std::string file_path_string = "../" + _dirName + "/" + _fileName + ".yaml";
        std::ofstream result(file_path_string);

        result << subgoal_generator_result;

        return true;
    }

    void Manager::exportDBData(const ExportDB &_exportDB, YAML::Node &_node)
    {
        for (const auto &exportDataPair : _exportDB)
        {
            const ExportDataContainer &exportData = exportDataPair.second;
            
            exportDataContainerData(exportData, _node);
        }
    }

    void Manager::exportDataContainerData(const ExportDataContainer &_exportData, YAML::Node &_node)
    {
        YAML::Node exportData;

        VelocityObstacle::Manager::exportAgentInfoData(_exportData.agent_, exportData);

        YAML::Node cell;
        BufferedVoronoiDiagram::Manager::exportVoronoiCellData(_exportData.voronoi_cell_, cell);
        BufferedVoronoiDiagram::Manager::exportPolygonData(_exportData.buffered_voronoi_cell_.second, "offset_polygon", cell);
        exportData["cell"] = cell;

        VelocityObstacle::Manager::exportAgentVOData(_exportData.agent_, exportData);

        _node.push_back(exportData);
    }

    bool Manager::isDirExists(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;
        std::filesystem::path directory_path(directory_path_string);

        return std::filesystem::exists(directory_path);
    }
} // namespace SubgoalGenerator
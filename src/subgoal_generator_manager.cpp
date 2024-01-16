#include "subgoal_generator/subgoal_generator_manager.h"

namespace SubgoalGenerator
{
    void Manager::clearResultDir(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;

        if (isDirExists(_dirName))
            std::filesystem::remove_all(directory_path_string);

        std::filesystem::create_directories(directory_path_string);
    }

    bool Manager::isDirExists(std::string _dirName)
    {
        std::string directory_path_string = "../" + _dirName;
        std::filesystem::path directory_path(directory_path_string);

        return std::filesystem::exists(directory_path);
    }
} // namespace SubgoalGenerator
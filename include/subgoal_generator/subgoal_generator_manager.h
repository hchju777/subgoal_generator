#pragma once

#include <string>
#include <map>
#include <fstream>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "subgoal_generator/subgoal_generator.h"

namespace SubgoalGenerator
{
    class Manager
    {
    public:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static void clearResultDir(std::string _dirName = "result");

    protected:
        // Todo: Make manager interface
        // Todo: In case for other managers, they also can clear result directory.
        static bool isDirExists(std::string _dirName);
    }; // class Manager
} // namespace SubgoalGenerator
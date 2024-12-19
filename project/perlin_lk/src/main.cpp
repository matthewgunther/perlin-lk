#include <boost/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "engine.h"

/// @brief Get the config file path from command line arguments, throw exception
/// if not given
/// @param argc Integer number of arguments
/// @param argv Array of arguments
/// @return Path to config file
std::string get_config_path(int argc, char *argv[]) {

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "-c") {
      if (i + 1 < argc) {
        std::string config_path = argv[i + 1];
        return config_path;
      }
    }
  }
  throw std::runtime_error(
      "Error: Please provide a path to the config file in "
      "the format `./build/perlin_lk -c ./project/perlin_lk/config.json`");
}

/// @brief Main function to run engine
/// @param argc Integer number of arguments
/// @param argv Array of arguments
/// @return Integer of exit code
int main(int argc, char *argv[]) {
  std::ifstream file(get_config_path(argc, argv));
  boost::json::value config = boost::json::parse(file);
  Engine engine(config);
  engine.run();
}

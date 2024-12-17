#include "engine.h"
#include <fstream>
#include <iostream>

#include <boost/json.hpp>
#include <filesystem>

std::string get_config_path(int argc, char *argv[]) {

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "-c") {
      if (i + 1 < argc) {
        std::string config_path = argv[i + 1];
        return config_path;
      }
    }
  }
  throw std::runtime_error("Error: Please provide a path to the config file in "
                           "the format `./perlin_lk -c /path/to/config.json`");
}

int main(int argc, char *argv[]) {
  std::ifstream file(get_config_path(argc, argv));
  boost::json::value config = boost::json::parse(file);
  Engine engine(config);
  engine.run();
}
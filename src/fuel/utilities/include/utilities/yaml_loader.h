#pragma once

#include "common_types.h"
#include <yaml-cpp/yaml.h>
#include <string>

namespace utilities
{
  AllParams loadParamsFromYaml(const std::string &file_path);
  void loadSysParams(AllParams &params, const YAML::Node &config);
  void loadConstraints(AllParams &params, const YAML::Node &config);
  void solveLQR(AllParams &params,
                double tolerance = 1e-6,
                int max_iter = 1000);
  void computeMPCParams(AllParams &params);
  void loadMPCParams(AllParams &params, const YAML::Node &config);
  void loadGovernorParams(AllParams &params, const YAML::Node &config);
} // namespace utilities

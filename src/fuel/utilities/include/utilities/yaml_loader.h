#pragma once

#include "common_types.h"
#include <yaml-cpp/yaml.h>
#include <string>

namespace utilities
{
  AllParams loadParamsFromYaml(const std::string &file_path);

  void loadSysParams(AllParams &params, const YAML::Node &config);

  void loadConstraints(AllParams &params, const YAML::Node &config);

  void loadMPCParams(AllParams &params, const YAML::Node &config);

  void loadGovernorParams(AllParams &params, const YAML::Node &config);

} // namespace utilities

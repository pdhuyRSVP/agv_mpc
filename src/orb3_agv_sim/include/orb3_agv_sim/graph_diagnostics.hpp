#pragma once

#include <map>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace orb3_agv_sim
{

struct GraphTiming
{
  double build_graph_sec{0.0};
  double optimize_sec{0.0};
};

struct GraphDiagnosticsResult
{
  bool success{false};
  double linearize_sec{0.0};
  double diagnostics_sec{0.0};
  std::vector<std::string> warnings;
};

GraphDiagnosticsResult runGraphDiagnostics(
  const gtsam::NonlinearFactorGraph & graph,
  const gtsam::Values & initial_values,
  const gtsam::Values & result,
  const std::map<size_t, std::string> & factor_types,
  const std::string & output_dir = "diagnostics",
  const GraphTiming & timing = {});

}  // namespace orb3_agv_sim

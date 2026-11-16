#include <filesystem>
#include <iostream>
#include <map>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "orb3_agv_sim/graph_diagnostics.hpp"

int main(int argc, char ** argv)
{
  const std::string output_dir = argc > 1 ? argv[1] : "/tmp/graph_diagnostics_smoke";
  std::filesystem::remove_all(output_dir);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  std::map<size_t, std::string> factor_types;
  const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.05));
  const auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
  const auto map_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.15));

  const auto x0 = gtsam::Symbol('x', 0);
  const auto x1 = gtsam::Symbol('x', 1);
  const auto x2 = gtsam::Symbol('x', 2);
  initial.insert(x0, gtsam::Pose2(0.1, -0.1, 0.02));
  initial.insert(x1, gtsam::Pose2(1.1, 0.1, 0.04));
  initial.insert(x2, gtsam::Pose2(2.1, 0.4, 0.18));

  graph.add(gtsam::PriorFactor<gtsam::Pose2>(x0, gtsam::Pose2(), prior_noise));
  factor_types[graph.size() - 1] = "prior";
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
    x0, x1, gtsam::Pose2(1.0, 0.0, 0.0), odom_noise));
  factor_types[graph.size() - 1] = "ekf";
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
    x1, x2, gtsam::Pose2(1.0, 0.2, 0.15), odom_noise));
  factor_types[graph.size() - 1] = "orb";
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
    x0, x2, gtsam::Pose2(2.0, 0.2, 0.15), odom_noise));
  factor_types[graph.size() - 1] = "scan_odom";
  graph.add(gtsam::PriorFactor<gtsam::Pose2>(
    x2, gtsam::Pose2(2.0, 0.2, 0.15), map_noise));
  factor_types[graph.size() - 1] = "scan_map";

  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  const auto report = orb3_agv_sim::runGraphDiagnostics(
    graph, initial, result, factor_types, output_dir, {0.001, 0.002});
  std::cout << "success=" << report.success << " warnings=" << report.warnings.size() << '\n';
  return report.success ? 0 : 1;
}

#include "orb3_agv_sim/graph_diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace orb3_agv_sim
{
namespace
{

using Clock = std::chrono::steady_clock;
using Counts = std::map<std::string, size_t>;

constexpr const char * kTypes[] = {"prior", "ekf", "orb", "scan_odom", "scan_map"};

struct ResidualStats
{
  size_t count{0};
  double total{0.0};
  double mean{0.0};
  double maximum{0.0};
  double stddev{0.0};
};

struct PointRecord
{
  gtsam::Key key;
  double x;
  double y;
  bool landmark;
};

double secondsSince(const Clock::time_point & start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

std::string factorType(
  const std::map<size_t, std::string> & factor_types, size_t index)
{
  const auto it = factor_types.find(index);
  return it == factor_types.end() ? "unknown" : it->second;
}

Counts computeFactorCounts(
  const gtsam::NonlinearFactorGraph & graph,
  const std::map<size_t, std::string> & factor_types)
{
  Counts counts;
  for (const char * type : kTypes) {
    counts[type] = 0;
  }
  counts["unknown"] = 0;
  for (size_t i = 0; i < graph.size(); ++i) {
    ++counts[factorType(factor_types, i)];
  }
  return counts;
}

bool isPoseKey(gtsam::Key key)
{
  return gtsam::Symbol(key).chr() == 'x';
}

std::map<gtsam::Key, size_t> computeNodeDegrees(
  const gtsam::NonlinearFactorGraph & graph,
  const gtsam::Values & result)
{
  std::map<gtsam::Key, size_t> degrees;
  for (const gtsam::Key key : result.keys()) {
    if (isPoseKey(key)) {
      degrees[key] = 0;
    }
  }
  for (const auto & factor : graph) {
    if (!factor) {
      continue;
    }
    for (const gtsam::Key key : factor->keys()) {
      auto it = degrees.find(key);
      if (it != degrees.end()) {
        ++it->second;
      }
    }
  }
  return degrees;
}

std::map<std::string, std::vector<std::pair<size_t, double>>> computeResidualsByType(
  const gtsam::NonlinearFactorGraph & graph,
  const gtsam::Values & result,
  const std::map<size_t, std::string> & factor_types,
  std::vector<std::string> & warnings)
{
  std::map<std::string, std::vector<std::pair<size_t, double>>> residuals;
  for (const char * type : kTypes) {
    residuals[type] = {};
  }
  residuals["unknown"] = {};

  for (size_t i = 0; i < graph.size(); ++i) {
    const auto factor = graph.at(i);
    if (!factor) {
      continue;
    }
    try {
      residuals[factorType(factor_types, i)].emplace_back(i, factor->error(result));
    } catch (const std::exception & e) {
      warnings.push_back(
        "factor " + std::to_string(i) + " residual failed: " + e.what());
    }
  }
  return residuals;
}

ResidualStats summarize(const std::vector<std::pair<size_t, double>> & values)
{
  ResidualStats stats;
  stats.count = values.size();
  if (values.empty()) {
    return stats;
  }
  for (const auto & item : values) {
    stats.total += item.second;
    stats.maximum = std::max(stats.maximum, item.second);
  }
  stats.mean = stats.total / static_cast<double>(stats.count);
  double variance = 0.0;
  for (const auto & item : values) {
    const double delta = item.second - stats.mean;
    variance += delta * delta;
  }
  stats.stddev = std::sqrt(variance / static_cast<double>(stats.count));
  return stats;
}

std::map<std::string, ResidualStats> summarizeResiduals(
  const std::map<std::string, std::vector<std::pair<size_t, double>>> & residuals)
{
  std::map<std::string, ResidualStats> output;
  for (const char * type : kTypes) {
    const auto it = residuals.find(type);
    output[type] = it == residuals.end() ? ResidualStats{} : summarize(it->second);
  }
  return output;
}

std::vector<PointRecord> collectPoints(
  const gtsam::Values & result, std::vector<std::string> & warnings)
{
  std::vector<PointRecord> points;
  for (const gtsam::Key key : result.keys()) {
    const char symbol = gtsam::Symbol(key).chr();
    try {
      if (symbol == 'x') {
        const auto pose = result.at<gtsam::Pose2>(key);
        points.push_back({key, pose.x(), pose.y(), false});
      } else if (symbol == 'l') {
        const auto point = result.at<gtsam::Point2>(key);
        points.push_back({key, point.x(), point.y(), true});
      }
    } catch (const std::exception & e) {
      warnings.push_back(
        "variable " + gtsam::DefaultKeyFormatter(key) + " cannot be plotted: " + e.what());
    }
  }
  return points;
}

cv::Scalar edgeColor(const std::string & type)
{
  if (type == "prior") return {180, 40, 180};
  if (type == "ekf") return {40, 160, 40};
  if (type == "orb") return {220, 120, 20};
  if (type == "scan_odom") return {30, 180, 230};
  if (type == "scan_map") return {30, 30, 220};
  return {130, 130, 130};
}

void plotFactorGraph(
  const gtsam::NonlinearFactorGraph & graph,
  const gtsam::Values & result,
  const std::map<size_t, std::string> & factor_types,
  const std::filesystem::path & path,
  std::vector<std::string> & warnings)
{
  const auto points = collectPoints(result, warnings);
  if (points.empty()) {
    warnings.push_back("factor graph plot skipped: no Pose2/Point2 values");
    return;
  }

  std::map<gtsam::Key, PointRecord> by_key;
  double min_x = points.front().x, max_x = points.front().x;
  double min_y = points.front().y, max_y = points.front().y;
  for (const auto & point : points) {
    by_key[point.key] = point;
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  if (std::fabs(max_x - min_x) < 1e-6) {
    min_x -= 1.0;
    max_x += 1.0;
  }
  if (std::fabs(max_y - min_y) < 1e-6) {
    min_y -= 1.0;
    max_y += 1.0;
  }

  constexpr int width = 1500;
  constexpr int height = 1000;
  constexpr int margin = 80;
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
  auto pixel = [&](const PointRecord & point) {
      const double px = margin + (point.x - min_x) / (max_x - min_x) * (width - 2 * margin);
      const double py = height - margin -
        (point.y - min_y) / (max_y - min_y) * (height - 2 * margin);
      return cv::Point(static_cast<int>(px), static_cast<int>(py));
    };

  for (size_t i = 0; i < graph.size(); ++i) {
    const auto factor = graph.at(i);
    if (!factor) continue;
    std::vector<PointRecord> connected;
    for (const gtsam::Key key : factor->keys()) {
      const auto it = by_key.find(key);
      if (it != by_key.end()) connected.push_back(it->second);
    }
    for (size_t j = 1; j < connected.size(); ++j) {
      cv::line(
        image, pixel(connected[j - 1]), pixel(connected[j]),
        edgeColor(factorType(factor_types, i)), 2, cv::LINE_AA);
    }
    if (connected.size() == 1) {
      cv::circle(image, pixel(connected.front()), 11, edgeColor(factorType(factor_types, i)), 2);
    }
  }

  for (const auto & point : points) {
    const cv::Point p = pixel(point);
    if (point.landmark) {
      const cv::Point diamond[] = {
        {p.x, p.y - 7}, {p.x + 7, p.y}, {p.x, p.y + 7}, {p.x - 7, p.y}};
      cv::fillConvexPoly(image, diamond, 4, cv::Scalar(20, 20, 20), cv::LINE_AA);
    } else {
      cv::circle(image, p, 5, cv::Scalar(30, 30, 30), cv::FILLED, cv::LINE_AA);
    }
  }

  int legend_y = 30;
  for (const char * type : kTypes) {
    cv::line(image, {25, legend_y}, {65, legend_y}, edgeColor(type), 4, cv::LINE_AA);
    cv::putText(
      image, type, {75, legend_y + 5}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
      cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
    legend_y += 27;
  }
  cv::putText(
    image, "Pose nodes: circles; landmarks: diamonds", {25, height - 25},
    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
  if (!cv::imwrite(path.string(), image)) {
    throw std::runtime_error("cannot write " + path.string());
  }
}

void plotMatrixSparsity(const gtsam::Matrix & matrix, const std::filesystem::path & path)
{
  const int rows = static_cast<int>(matrix.rows());
  const int cols = static_cast<int>(matrix.cols());
  if (rows == 0 || cols == 0) {
    throw std::runtime_error("empty matrix");
  }
  cv::Mat binary(rows, cols, CV_8UC1, cv::Scalar(255));
  const double epsilon = 1e-12;
  for (int row = 0; row < rows; ++row) {
    auto * pixels = binary.ptr<unsigned char>(row);
    for (int col = 0; col < cols; ++col) {
      if (std::fabs(matrix(row, col)) > epsilon) {
        pixels[col] = 0;
      }
    }
  }
  const double scale = std::min(1.0, 1400.0 / std::max(rows, cols));
  cv::Mat output;
  if (scale < 1.0) {
    cv::resize(binary, output, {}, scale, scale, cv::INTER_AREA);
  } else {
    const double enlarge = std::min(8.0, 900.0 / std::max(rows, cols));
    cv::resize(binary, output, {}, enlarge, enlarge, cv::INTER_NEAREST);
  }
  cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
  if (!cv::imwrite(path.string(), output)) {
    throw std::runtime_error("cannot write " + path.string());
  }
}

void plotDegreeHistogram(
  const std::map<gtsam::Key, size_t> & degrees, const std::filesystem::path & path)
{
  std::map<size_t, size_t> histogram;
  for (const auto & item : degrees) ++histogram[item.second];
  cv::Mat image(700, 1100, CV_8UC3, cv::Scalar(255, 255, 255));
  if (!histogram.empty()) {
    const size_t max_count = std::max_element(
      histogram.begin(), histogram.end(),
      [](const auto & a, const auto & b) {return a.second < b.second;})->second;
    const int bar_width = std::max(20, 900 / static_cast<int>(histogram.size()));
    int index = 0;
    for (const auto & item : histogram) {
      const int x = 90 + index * bar_width;
      const int bar_height = static_cast<int>(500.0 * item.second / std::max<size_t>(1, max_count));
      cv::rectangle(
        image, {x, 600 - bar_height}, {x + bar_width - 8, 600},
        cv::Scalar(70, 130, 210), cv::FILLED);
      cv::putText(
        image, std::to_string(item.first), {x, 630},
        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(20, 20, 20), 1);
      cv::putText(
        image, std::to_string(item.second), {x, 585 - bar_height},
        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(20, 20, 20), 1);
      ++index;
    }
  }
  cv::putText(image, "Pose node degree", {430, 675}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {20, 20, 20}, 1);
  if (!cv::imwrite(path.string(), image)) throw std::runtime_error("cannot write histogram");
}

void plotResidualBars(
  const std::map<std::string, ResidualStats> & summaries,
  const std::filesystem::path & path)
{
  cv::Mat image(700, 1100, CV_8UC3, cv::Scalar(255, 255, 255));
  double max_mean = 0.0;
  for (const char * type : kTypes) max_mean = std::max(max_mean, summaries.at(type).mean);
  int index = 0;
  for (const char * type : kTypes) {
    const auto & stats = summaries.at(type);
    const int x = 100 + index * 190;
    const int bar_height = max_mean > 0.0 ? static_cast<int>(480.0 * stats.mean / max_mean) : 0;
    cv::rectangle(
      image, {x, 590 - bar_height}, {x + 105, 590},
      edgeColor(type), cv::FILLED);
    cv::putText(image, type, {x, 625}, cv::FONT_HERSHEY_SIMPLEX, 0.46, {20, 20, 20}, 1);
    std::ostringstream label;
    label << std::setprecision(3) << stats.mean;
    cv::putText(image, label.str(), {x, 575 - bar_height}, cv::FONT_HERSHEY_SIMPLEX, 0.42, {20, 20, 20}, 1);
    ++index;
  }
  cv::putText(image, "Mean factor error", {430, 675}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {20, 20, 20}, 1);
  if (!cv::imwrite(path.string(), image)) throw std::runtime_error("cannot write residual plot");
}

void writeFactorCounts(const Counts & counts, size_t total, const std::filesystem::path & path)
{
  std::ofstream out(path);
  out << "metric,value\n";
  out << "num_prior_factors," << counts.at("prior") << '\n';
  out << "num_ekf_factors," << counts.at("ekf") << '\n';
  out << "num_orb_factors," << counts.at("orb") << '\n';
  out << "num_scan_odom_factors," << counts.at("scan_odom") << '\n';
  out << "num_scan_map_factors," << counts.at("scan_map") << '\n';
  out << "total_factors," << total << '\n';
}

void writeDegrees(
  const std::map<gtsam::Key, size_t> & degrees, const std::filesystem::path & path)
{
  std::ofstream out(path);
  out << "key,degree\n";
  for (const auto & item : degrees) {
    out << gtsam::DefaultKeyFormatter(item.first) << ',' << item.second << '\n';
  }
}

void writeResiduals(
  const std::map<std::string, std::vector<std::pair<size_t, double>>> & residuals,
  const std::map<std::string, ResidualStats> & summaries,
  const std::filesystem::path & output_dir)
{
  std::ofstream per_factor(output_dir / "residual_per_factor.csv");
  per_factor << "factor_index,factor_type,error\n";
  for (const auto & group : residuals) {
    for (const auto & item : group.second) {
      per_factor << item.first << ',' << group.first << ',' << std::setprecision(17) <<
        item.second << '\n';
    }
  }

  std::ofstream summary(output_dir / "residual_summary.csv");
  summary << "factor_type,count,total,mean,max,std\n";
  for (const char * type : kTypes) {
    const auto & stats = summaries.at(type);
    summary << type << ',' << stats.count << ',' << std::setprecision(17) <<
      stats.total << ',' << stats.mean << ',' << stats.maximum << ',' << stats.stddev << '\n';
  }
}

std::tuple<double, double, double, double> degreeStats(
  const std::map<gtsam::Key, size_t> & degrees)
{
  if (degrees.empty()) return {0.0, 0.0, 0.0, 0.0};
  std::vector<double> values;
  values.reserve(degrees.size());
  for (const auto & item : degrees) values.push_back(static_cast<double>(item.second));
  std::sort(values.begin(), values.end());
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  const size_t middle = values.size() / 2;
  const double median = values.size() % 2 == 0 ?
    (values[middle - 1] + values[middle]) / 2.0 : values[middle];
  return {values.front(), values.back(), mean, median};
}

void saveMetricsJson(
  const std::filesystem::path & path,
  size_t num_variables,
  size_t num_factors,
  double initial_error,
  double final_error,
  const Counts & counts,
  const std::tuple<double, double, double, double> & degree,
  const std::map<std::string, ResidualStats> & residuals,
  const GraphTiming & timing,
  double linearize_sec,
  double diagnostics_sec,
  const std::vector<std::string> & warnings)
{
  const double reduction = std::fabs(initial_error) > 1e-15 ?
    100.0 * (initial_error - final_error) / initial_error : 0.0;
  std::ofstream out(path);
  out << std::setprecision(17);
  out << "{\n";
  out << "  \"num_variables\": " << num_variables << ",\n";
  out << "  \"num_factors\": " << num_factors << ",\n";
  out << "  \"initial_error\": " << initial_error << ",\n";
  out << "  \"final_error\": " << final_error << ",\n";
  out << "  \"error_reduction_percent\": " << reduction << ",\n";
  out << "  \"factor_counts\": {";
  for (size_t i = 0; i < std::size(kTypes); ++i) {
    if (i) out << ", ";
    out << "\"" << kTypes[i] << "\": " << counts.at(kTypes[i]);
  }
  out << "},\n";
  out << "  \"degree\": {\"min\": " << std::get<0>(degree) <<
    ", \"max\": " << std::get<1>(degree) << ", \"mean\": " <<
    std::get<2>(degree) << ", \"median\": " << std::get<3>(degree) << "},\n";
  out << "  \"residuals\": {\n";
  for (size_t i = 0; i < std::size(kTypes); ++i) {
    const auto & stats = residuals.at(kTypes[i]);
    out << "    \"" << kTypes[i] << "\": {\"count\": " << stats.count <<
      ", \"total\": " << stats.total << ", \"mean\": " << stats.mean <<
      ", \"max\": " << stats.maximum << ", \"std\": " << stats.stddev << "}";
    out << (i + 1 == std::size(kTypes) ? "\n" : ",\n");
  }
  out << "  },\n";
  out << "  \"timing\": {\"build_graph_sec\": " << timing.build_graph_sec <<
    ", \"optimize_sec\": " << timing.optimize_sec <<
    ", \"linearize_sec\": " << linearize_sec <<
    ", \"diagnostics_sec\": " << diagnostics_sec << "},\n";
  out << "  \"warnings\": [";
  for (size_t i = 0; i < warnings.size(); ++i) {
    if (i) out << ", ";
    std::string text = warnings[i];
    std::replace(text.begin(), text.end(), '"', '\'');
    out << "\"" << text << "\"";
  }
  out << "]\n}\n";
}

}  // namespace

GraphDiagnosticsResult runGraphDiagnostics(
  const gtsam::NonlinearFactorGraph & graph,
  const gtsam::Values & initial_values,
  const gtsam::Values & result,
  const std::map<size_t, std::string> & factor_types,
  const std::string & output_dir,
  const GraphTiming & timing)
{
  GraphDiagnosticsResult report;
  const auto diagnostics_start = Clock::now();
  const std::filesystem::path directory(output_dir);
  std::filesystem::create_directories(directory);

  if (graph.empty() || result.empty()) {
    report.warnings.push_back("diagnostics skipped: graph or result is empty");
    std::map<std::string, ResidualStats> empty_residuals;
    for (const char * type : kTypes) {
      empty_residuals[type] = {};
    }
    saveMetricsJson(
      directory / "graph_metrics.json", result.size(), graph.size(), 0.0, 0.0,
      computeFactorCounts(graph, factor_types), {0.0, 0.0, 0.0, 0.0}, empty_residuals,
      timing, 0.0, secondsSince(diagnostics_start), report.warnings);
    return report;
  }

  const Counts counts = computeFactorCounts(graph, factor_types);
  const auto degrees = computeNodeDegrees(graph, result);
  const auto degree_summary = degreeStats(degrees);
  const auto residual_values =
    computeResidualsByType(graph, result, factor_types, report.warnings);
  const auto residual_summaries = summarizeResiduals(residual_values);

  double initial_error = 0.0;
  double final_error = 0.0;
  try {
    initial_error = graph.error(initial_values);
  } catch (const std::exception & e) {
    report.warnings.push_back(std::string("initial error failed: ") + e.what());
  }
  try {
    final_error = graph.error(result);
  } catch (const std::exception & e) {
    report.warnings.push_back(std::string("final error failed: ") + e.what());
  }

  writeFactorCounts(counts, graph.size(), directory / "factor_counts.csv");
  writeDegrees(degrees, directory / "degree_per_pose.csv");
  writeResiduals(residual_values, residual_summaries, directory);

  try {
    plotFactorGraph(
      graph, result, factor_types, directory / "factor_graph.png", report.warnings);
  } catch (const std::exception & e) {
    report.warnings.push_back(std::string("factor graph plot failed: ") + e.what());
  }
  try {
    plotDegreeHistogram(degrees, directory / "degree_histogram.png");
    plotResidualBars(residual_summaries, directory / "residual_by_type_bar.png");
  } catch (const std::exception & e) {
    report.warnings.push_back(std::string("summary plot failed: ") + e.what());
  }

  const auto linearize_start = Clock::now();
  try {
    const auto linear_graph = graph.linearize(result);
    const auto jacobian = linear_graph->jacobian();
    const auto hessian = linear_graph->hessian();
    report.linearize_sec = secondsSince(linearize_start);
    plotMatrixSparsity(jacobian.first, directory / "jacobian_sparsity.png");
    plotMatrixSparsity(hessian.first, directory / "hessian_sparsity.png");
  } catch (const std::exception & e) {
    report.linearize_sec = secondsSince(linearize_start);
    report.warnings.push_back(std::string("linearize/sparsity failed: ") + e.what());
  }

  report.diagnostics_sec = secondsSince(diagnostics_start);
  saveMetricsJson(
    directory / "graph_metrics.json", result.size(), graph.size(),
    initial_error, final_error, counts, degree_summary, residual_summaries,
    timing, report.linearize_sec, report.diagnostics_sec, report.warnings);
  report.success = true;
  return report;
}

}  // namespace orb3_agv_sim

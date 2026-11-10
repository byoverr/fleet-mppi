// Stress test for NoiseGenerator shutdown with regenerate_noises = true: many short lifecycles
// (initialize, a few optimizer-like cycles, shutdown). A shutdown that does not return within
// 5 s is a hang (the process exits with code 3). Build against the stock or the fixed source.
// Usage: stress TRIALS [cycles_per_trial]
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_mppi_controller/tools/noise_generator.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(1, argv);
  const int trials = argc > 1 ? std::atoi(argv[1]) : 1000;
  const int cycles = argc > 2 ? std::atoi(argv[2]) : 3;
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(true));
  std::string name = "test";
  mppi::ParametersHandler handler(node, name);
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 32;
  settings.time_steps = 8;
  settings.sampling_std.vx = 0.2f;
  settings.sampling_std.vy = 0.2f;
  settings.sampling_std.wz = 0.4f;
  mppi::models::ControlSequence cs;
  cs.reset(settings.time_steps);
  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);
  for (int t = 0; t < trials; ++t) {
    mppi::NoiseGenerator g;
    g.initialize(settings, false, "test_name", &handler);
    for (int c = 0; c < cycles; ++c) {
      g.setNoisedControls(state, cs);
      g.generateNextNoises();
    }
    std::atomic<bool> done{false};
    std::thread watchdog([&] {
        for (int i = 0; i < 500 && !done; ++i) {std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        if (!done) {
          std::printf("HANG at trial %d\n", t);
          std::fflush(stdout);
          if (std::getenv("HOLD")) {std::this_thread::sleep_for(std::chrono::seconds(120));}
          std::_Exit(3);
        }
      });
    g.shutdown();
    done = true;
    watchdog.join();
  }
  std::printf("ok %d trials\n", trials);
  rclcpp::shutdown();
  return 0;
}

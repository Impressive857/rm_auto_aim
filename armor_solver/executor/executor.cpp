//
// Created by lbw on 25-3-10.
//
#include <register_node_macro.h>
#include <shared_memory.cpp>

#include <clock/../../src/clock.cpp>
#include <parameter/../../src/parameter.cpp>
#include <data_center/../../src/data_recorder.cpp>
#include <data_center/../../src/listener/ros_listener.cpp>
#include <armor_solver/../../src/armor_solver_node.cpp>
#include <visualization/../../src/visualization.cpp>

namespace global_node
{
  std::shared_ptr<ckyf::parameter::ArmorSolverParam> Parameter;
  std::shared_ptr<ckyf::data_center::DataRecorder> DataRecorder;
  std::shared_ptr<ckyf::auto_aim::ArmorSolverNode> ArmorSolver;
  std::shared_ptr<ckyf::visualization::Visualization> Visualization;
}

int main(int argc,char** argv) {
  rclcpp::init(argc, argv);
  load_factory::init::spinning = true;
  rclcpp::executors::MultiThreadedExecutor executor;
  load_factory::init::instantiation();
  load_factory::init::refresh_reference();
  std::cout << "Node list: [";
  for (auto &[name,node_ptr]:load_factory::map::node_map)
  {
    std::cout << name << ", ";
    executor.add_node(node_ptr);
  }
  std::cout << "]" << std::endl;
  executor.spin();
  rclcpp::shutdown();
}


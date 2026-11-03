// Diagnostic only: isolated ROS domain, synthetic flat terrain, no motion publisher.
#include <nav2_mppi_controller/optimizer.hpp>
#include <nav2_mppi_controller/tools/trajectory_visualizer.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <algorithm>
#include <numeric>

using Clock = std::chrono::steady_clock;
double ms(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b-a).count();
}
void report(const char *name, std::vector<double> a) {
  std::sort(a.begin(), a.end());
  std::cout << "RESULT " << name << " n=" << a.size()
    << " p50_ms=" << a[a.size()/2] << " p95_ms=" << a[a.size()*95/100]
    << " max_ms=" << a.back() << std::endl;
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("frequency_costmap", "", "frequency_costmap");
    costmap->set_parameter(rclcpp::Parameter("global_frame", "map"));
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->on_configure(rclcpp_lifecycle::State{});
    costmap->getCostmap()->resizeMap(140,140,.05,-3.5,-3.5);
    costmap->getCostmap()->resetMap(0,0,140,140);
    auto params=std::make_unique<mppi::ParametersHandler>(node);
    auto optimizer=std::make_unique<mppi::Optimizer>();
    optimizer->initialize(node,"FollowPath",costmap,params.get());
    mppi::TrajectoryVisualizer visualizer;
    visualizer.on_configure(node,"FollowPath","map",params.get());
    visualizer.on_activate();params->start();
    // Force actual DDS serialization, as when RViz/rosbag subscribes. All topics
    // remain in the isolated diagnostic domain; no cmd_vel publisher is created.
    size_t received=0;
    auto listener=std::make_shared<rclcpp::Node>("frequency_probe_listener");
    auto sub=listener->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/trajectories",10,[&](visualization_msgs::msg::MarkerArray::ConstSharedPtr){++received;});
    auto pub=node->create_publisher<grid_map_msgs::msg::GridMap>("/elevation_mapping_node/elevation_map",1);
    pub->on_activate();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());executor.add_node(listener);
    const auto deadline=Clock::now()+std::chrono::seconds(3);
    while(Clock::now()<deadline && pub->get_subscription_count()==0){
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    grid_map::GridMap map({"elevation"});map.setGeometry(grid_map::Length(7,7),.1);
    map.setFrameId("map");map["elevation"].setConstant(0);
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.pose.orientation.w=1;
    geometry_msgs::msg::Twist speed;speed.linear.x=.4;
    nav_msgs::msg::Path path;path.header.frame_id="map";
    for(int i=0;i<=60;++i){auto p=pose;p.pose.position.x=i*.05;path.poses.push_back(p);}
    nav2_controller::SimpleGoalChecker checker;checker.initialize(node,"precise_goal_checker",costmap);
    std::vector<double> compute,build,publish,total,intervals;
    bool periodic=false;node->get_parameter_or("frequency_probe.periodic",periodic,false);
    auto previous=Clock::now();
    bool show=true;node->get_parameter_or("FollowPath.visualize",show,true);
    for(int round=0;round<70;++round){
      const auto tick=Clock::now();
      pose.header.stamp=node->now();path.header.stamp=pose.header.stamp;
      map.setTimestamp(node->now().nanoseconds());pub->publish(*grid_map::GridMapRosConverter::toMessage(map));
      executor.spin_some();
      if(round==0){std::this_thread::sleep_for(std::chrono::milliseconds(20));executor.spin_some();}
      auto a=Clock::now();
      auto command=optimizer->evalControl(pose,speed,path,&checker);
      auto b=Clock::now();
      if(show){visualizer.add(optimizer->getGeneratedTrajectories(),"Candidate Trajectories");
        visualizer.add(optimizer->getOptimizedTrajectory(),"Optimal Trajectory");}
      auto c=Clock::now();if(show){visualizer.visualize(path);}auto d=Clock::now();
      if(round>=10){compute.push_back(ms(a,b));build.push_back(ms(b,c));publish.push_back(ms(c,d));total.push_back(ms(a,d));intervals.push_back(ms(previous,d));}
      previous=d;
      executor.spin_some();
      if(!std::isfinite(command.twist.linear.x)||!std::isfinite(command.twist.angular.z))throw std::runtime_error("invalid command");
      if(periodic)std::this_thread::sleep_until(tick+std::chrono::milliseconds(50));
    }
    report("completion_interval",intervals);report("optimizer",compute);report("visualizer_build",build);report("visualizer_publish",publish);report("compute_plus_visualization",total);
    std::cout<<"RESULT received_marker_arrays="<<received<<" visualize="<<show<<" periodic="<<periodic<<std::endl;
    visualizer.on_deactivate();visualizer.on_cleanup();optimizer.reset();params.reset();
    executor.remove_node(node->get_node_base_interface());executor.remove_node(listener);
    pub.reset();sub.reset();listener.reset();costmap->on_cleanup(rclcpp_lifecycle::State{});costmap.reset();node.reset();
    rclcpp::shutdown();return 0;
  }catch(const std::exception &e){std::cerr<<"ERROR "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
}

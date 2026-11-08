// Temporary observational interposer. Calls the original functions unchanged.
#include <nav2_mppi_controller/controller.hpp>
#include <dlfcn.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <stdexcept>
using Clock = std::chrono::steady_clock;
static void * original(const char *symbol) {
  static void *handle=dlopen("/opt/ros/humble/lib/libmppi_controller.so",RTLD_LAZY|RTLD_NOLOAD);
  if(!handle) throw std::runtime_error("MPPI timing: original library unavailable");
  void *address=dlsym(handle,symbol);
  if(!address) throw std::runtime_error("MPPI timing: original symbol unavailable");
  return address;
}
static void record(const char *stage, Clock::time_point start, bool ok) {
  static int fd=[] {const char *path=std::getenv("MPPI_PROFILE_LOG");return path?open(path,O_WRONLY|O_CREAT|O_APPEND|O_CLOEXEC,0644):-1;}();
  if(fd<0)return;
  auto end=Clock::now();
  const double start_s=std::chrono::duration<double>(start.time_since_epoch()).count();
  const double elapsed=std::chrono::duration<double,std::milli>(end-start).count();
  char line[160];int n=std::snprintf(line,sizeof(line),"%s,%.9f,%.6f,%d\n",stage,start_s,elapsed,ok);
  (void)!write(fd,line,n);
}
geometry_msgs::msg::TwistStamped nav2_mppi_controller::MPPIController::computeVelocityCommands(
 const geometry_msgs::msg::PoseStamped &p,const geometry_msgs::msg::Twist &v,nav2_core::GoalChecker *g) {
 using F=geometry_msgs::msg::TwistStamped(*)(MPPIController*,const geometry_msgs::msg::PoseStamped&,const geometry_msgs::msg::Twist&,nav2_core::GoalChecker*);
 static F fn=reinterpret_cast<F>(original("_ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE"));
 auto start=Clock::now();try{auto result=fn(this,p,v,g);record("controller",start,true);return result;}catch(...){record("controller",start,false);throw;}
}
geometry_msgs::msg::TwistStamped mppi::Optimizer::evalControl(
 const geometry_msgs::msg::PoseStamped &p,const geometry_msgs::msg::Twist &v,const nav_msgs::msg::Path &path,nav2_core::GoalChecker *g) {
 using F=geometry_msgs::msg::TwistStamped(*)(Optimizer*,const geometry_msgs::msg::PoseStamped&,const geometry_msgs::msg::Twist&,const nav_msgs::msg::Path&,nav2_core::GoalChecker*);
 static F fn=reinterpret_cast<F>(original("_ZN4mppi9Optimizer11evalControlERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EERKN8nav_msgs3msg5Path_IS4_EEPN9nav2_core11GoalCheckerE"));
 auto start=Clock::now();try{auto result=fn(this,p,v,path,g);record("optimizer",start,true);return result;}catch(...){record("optimizer",start,false);throw;}
}

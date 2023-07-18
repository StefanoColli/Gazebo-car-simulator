#include "ode_simulator/ode_simulator.h"

#define NODE_NAME "ode_simulator"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  
  OdeSimulator ode_simulator;
     
  ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

  // Wait other nodes start
  sleep(1.0);

  ode_simulator.Prepare();

  while (ros::ok())
  {
      ode_simulator.RunPeriodically();

      ros::spinOnce();

      usleep(1000);
  }

  ode_simulator.Shutdown();
  
  return (0);
}


#include "trajectory_tracker/trajectory_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  trajectory_tracker trajectory_tracker_node;
   
  trajectory_tracker_node.Prepare();
  
  trajectory_tracker_node.RunPeriodically(trajectory_tracker_node.RunPeriod);
  
  trajectory_tracker_node.Shutdown();
  
  return(0);
}

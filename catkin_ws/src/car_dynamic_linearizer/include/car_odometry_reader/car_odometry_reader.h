#ifndef CAR_ODOMETRY_READER_H
#define CAR_ODOMETRY_READER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

/**
 * @class CarOdometryReader
 * 
 * Read Odometry data from a topic.
 * Performs manipulation over Odometry data. 
 * Can simply show data or convert them in another format to be published in a different topic.
 * In general it's an interface between odometry data and Controller data format.
 * This class is straightly connected with the future controller.
 */
class CarOdometryReader
{
public:
    CarOdometryReader(ros::NodeHandle n);

    /**
     * @brief Mainly set topics to publish or to subscribe.
     * 
     */
    void prepare();

    /**
     * @brief Action executed at every ROS Spin.
     * 
     * Not implemented 
     */
    void runPeriodically();
    void shutDown();

private:
    /**
     * @brief Display received Odometry data (position, orientation and twist)
     * 
     * @param odom_msg
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    ros::NodeHandle node_handle_;
    ros::Subscriber sub_odometry_;
};

#endif /* CAR_ODOMETRY_READER_H */
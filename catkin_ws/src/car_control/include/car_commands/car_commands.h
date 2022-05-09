#ifndef CAR_COMMANDS_H
#define CAR_COMMANDS_H

#include "ros/ros.h"

/**
 * @class CarCommands
 * 
 * Publish commands to a topic nedded to make a car move. 
 * Commands have a predefined format. This class is not made to be extensible.
 * Specify a criterion to send commands to the car.
 */
class CarCommands
{
public:
    CarCommands(ros::NodeHandle n);

    /**
     * @brief Mainly set topics to publish or to subscribe.
     * 
     */
    void prepare();

    /**
     * @brief Action executed at every ROS Spin.
     * 
     * Publish commands to follow a straight line. 
     */
    void runPeriodically();
    void shutDown();

private:

    ros::NodeHandle node_handle_;
    ros::Publisher pub_command_;
    //static constexpr float INITIAL_SPEED {1.0f};
    //static constexpr float INITIAL_TURN {0.5f};
    float speed_;
    float turn_;
    int direction_speed_; /**< can take -1, 0, 1 depending on direction of the car (0 means "stop") */
    int direction_theta_; /**< can take -1, 0, 1 depending on the steering direction of the car (0 means "fixed dir.") */
};

#endif /* CAR_COMMANDS_H */
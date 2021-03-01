#pragma once

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <centauro_contact_detection/Contacts.h>

namespace contact_detection {

class ContactState {

public:
    ContactState(ros::NodeHandle& nodeHandle, int frequency);

    void spin();

    virtual ~ContactState();

private:
    ros::NodeHandle nodeHandle_;

    // Subscribers
    std::vector<ros::Subscriber> force_sub_;

    // Publishers
    int frequency_;
    ros::Publisher contact_pub_;

    // constants - variables
    float force_thold_;  // force threshold
    int level_;  // level of threshold violations in time window

    // flags
    std::vector<std::vector<bool>> contacts_flag_;
    std::vector<bool> contacts_;

    // methods - callbacks
    void ForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& force_msg,
                       int foot_idx);
};

}   //namespace

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <centauro_contact_detection/contacts.h>

namespace contact_detection {

class ContactState {

public:
    ContactState(ros::NodeHandle& nodeHandle, int frequency);

    virtual ~ContactState();

private:
    ros::NodeHandle nodeHandle_;

    // Subscribers
    ros::Subscriber fl_force_sub_;
    ros::Subscriber fr_force_sub_;
    ros::Subscriber hl_force_sub_;
    ros::Subscriber hr_force_sub_;

    // Publishers
    ros::Publisher fl_contact_pub_;

    // constants - variables
    float force_thold;  // force threshold
    int level;  // level of threshold violations in time window

    // flags
    bool contacts_flag[4][5] = {{false, false, false, false, true}, {false, false, false, false, false}, {false, false, false, false, false}, {false, false, false, false, false}};
    bool contacts[4] = {false, false, false, false};

    // methods - callbacks
    void ForceCallback1(const geometry_msgs::WrenchStamped::ConstPtr& force_msg);
    void ForceCallback2(const geometry_msgs::WrenchStamped::ConstPtr& force_msg);
    void ForceCallback3(const geometry_msgs::WrenchStamped::ConstPtr& force_msg);
    void ForceCallback4(const geometry_msgs::WrenchStamped::ConstPtr& force_msg);
};

}   //namespace

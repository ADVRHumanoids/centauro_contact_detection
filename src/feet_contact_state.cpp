#include <ros/ros.h>
#include <centauro_contact_detection/Feet_contact_state.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feet_contacts_node"); // Initialize ros
    ros::NodeHandle nodeHandle("~");    //create nodehandle

    contact_detection::ContactState contact_state(nodeHandle); // Create a class object

    ROS_INFO("\n\n-----------Out of constructor, ready to spin-----------");

    //ros::spin();    // Call any callback function waiting

    return 0;
}

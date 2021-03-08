#include <ros/ros.h>
#include <centauro_contact_detection/Feet_contact_state.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feet_contacts_node"); // Initialize ros
    ros::NodeHandle nodeHandle("~");    //create nodehandle

    int pub_freq;   // publish frequency
    nodeHandle.getParam("/feet_contacts_node/pub_freq", pub_freq);

    contact_detection::ContactState contact_state(nodeHandle, pub_freq); // Create a class object

    ROS_INFO("\n\n-----------Out of constructor, ready to spin-----------");

    contact_state.spin();

    //ros::spin();    // Call any callback function waiting

    return 0;
}

#include <centauro_contact_detection/Feet_contact_state.hpp>

namespace contact_detection {

ContactState::ContactState(ros::NodeHandle& nodeHandle, int frequency):
    nodeHandle_(nodeHandle),
    frequency_(frequency),
    contacts_(4, false),
    contacts_flag_(4, std::vector<bool>(5, false))
{
    // force subscribers
    for(int i = 0; i < 4; i++)
    {
        std::string topicname = "/cartesian/force_estimation/contact_" + std::to_string(i+1);

        auto sub = nodeHandle_.subscribe<geometry_msgs::WrenchStamped>(
                    topicname,
                    10,
                    [this, i](const geometry_msgs::WrenchStamped::ConstPtr& msg)
                    {
                        ForceCallback(msg, i);
                    });
    }

    // publisher of contacts
    contact_pub_ = nodeHandle_.advertise<centauro_contact_detection::Contacts>("/contacts", 10);

    // force threshold
    nodeHandle.getParam("/feet_contacts_node/force_thold", force_thold_);

}

void ContactState::spin()
{
    // publish frequency
    ros::Rate rate(frequency_);

    // message to publish
    centauro_contact_detection::Contacts contacts_msg;

    while(ros::ok())
    {
        ros::spinOnce();

        // assign values
        contacts_msg.f_left.data = contacts_[0];
        contacts_msg.f_right.data = contacts_[1];
        contacts_msg.h_left.data = contacts_[2];
        contacts_msg.h_right.data = contacts_[3];

        // publish
        contact_pub_.publish(contacts_msg);

        rate.sleep();
    }
}

ContactState::~ContactState() {}

void ContactState::ForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& force_msg,
                                 int foot_idx)
{
    // check force value
    if (force_msg->wrench.force.z >= force_thold_)
    {
        // find level of threshold violation in time window
        level_ = std::distance(contacts_flag_[foot_idx].begin(),
                               std::find(std::begin(contacts_flag_[foot_idx]),
                                         std::end(contacts_flag_[foot_idx]), true));

        if (level_ == 0) // force greater than threshold during time window
            contacts_[foot_idx] = true;

        else
            contacts_flag_[foot_idx][level_-1] = true;
    }

    else
    {
        contacts_[foot_idx] = false;    // loss of contact

        for (int i=0; i<5; i++)
            contacts_flag_[foot_idx][i] = false;
    }
}


} //namespace

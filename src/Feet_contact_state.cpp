#include <centauro_contact_detection/Feet_contact_state.hpp>

namespace contact_detection {

ContactState::ContactState(ros::NodeHandle& nodeHandle, int frequency) :
    nodeHandle_(nodeHandle)
{
    // force subscribers
    fl_force_sub_ = nodeHandle_.subscribe("/cartesian/force_estimation/contact_1", 10, &ContactState::ForceCallback1, this);
    fr_force_sub_ = nodeHandle_.subscribe("/cartesian/force_estimation/contact_2", 10, &ContactState::ForceCallback2, this);
    hl_force_sub_ = nodeHandle_.subscribe("/cartesian/force_estimation/contact_3", 10, &ContactState::ForceCallback3, this);
    hr_force_sub_ = nodeHandle_.subscribe("/cartesian/force_estimation/contact_4", 10, &ContactState::ForceCallback4, this);

    // publisher of contacts
    fl_contact_pub_ = nodeHandle_.advertise<centauro_contact_detection::contacts>("/contacts", 10);

    // force threshold
    nodeHandle.getParam("/feet_contacts_node/force_thold", force_thold);

    // message to publish
    centauro_contact_detection::contacts contacts_msg;

    // publish frequency
    ros::Rate rate(frequency);

    while (true)
    {
        ros::spinOnce();

        // assign values
        contacts_msg.f_left.data = contacts[0];
        contacts_msg.f_right.data = contacts[1];
        contacts_msg.h_left.data = contacts[2];
        contacts_msg.h_right.data = contacts[3];

        // publish
        fl_contact_pub_.publish(contacts_msg);

        rate.sleep();
    }
}

ContactState::~ContactState() {}

void ContactState::ForceCallback1(const geometry_msgs::WrenchStamped::ConstPtr& force_msg)
{
    // check force value
    if (force_msg->wrench.force.z >= force_thold)
    {
        // find level of threshold violation in time window
        level = std::distance(contacts_flag[0], std::find(std::begin(contacts_flag[0]), std::end(contacts_flag[0]), true));

        if (level == 0) // force greater than threshold during time window
            contacts[0] = true;

        else
            contacts_flag[0][level-1] = true;
    }

    else
    {
        contacts[0] = false;    // loss of contact

        for (int i=0; i<5; i++)
            contacts_flag[0][i] = false;
    }
}

void ContactState::ForceCallback2(const geometry_msgs::WrenchStamped::ConstPtr& force_msg)
{
    if (force_msg->wrench.force.z >= force_thold)
    {
        level = std::distance(contacts_flag[1], std::find(std::begin(contacts_flag[1]), std::end(contacts_flag[1]), true));

        if (level == 0)
            contacts[1] = true;

        else
            contacts_flag[1][level-1] = true;
    }

    else
    {
        contacts[1] = false;

        for (int i=0; i<5; i++)
            contacts_flag[1][i] = false;
    }
}

void ContactState::ForceCallback3(const geometry_msgs::WrenchStamped::ConstPtr& force_msg)
{
    if (force_msg->wrench.force.z >= force_thold)
    {
        level = std::distance(contacts_flag[2], std::find(std::begin(contacts_flag[2]), std::end(contacts_flag[2]), true));

        if (level == 0)
            contacts[2] = true;

        else
            contacts_flag[2][level-1] = true;
    }

    else
    {
        contacts[2] = false;

        for (int i=0; i<5; i++)
            contacts_flag[2][i] = false;
    }
}

void ContactState::ForceCallback4(const geometry_msgs::WrenchStamped::ConstPtr& force_msg)
{
    if (force_msg->wrench.force.z >= force_thold)
    {
        level = std::distance(contacts_flag[3], std::find(std::begin(contacts_flag[3]), std::end(contacts_flag[3]), true));

        if (level == 0)
            contacts[3] = true;

        else
            contacts_flag[3][level-1] = true;

    }

    else
    {
        contacts[3] = false;

        for (int i=0; i<5; i++)
            contacts_flag[3][i] = false;
    }
}

} //namespace

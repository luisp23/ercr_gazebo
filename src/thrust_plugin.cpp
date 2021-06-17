#include <thrust_plugin.h>


namespace gazebo {

    ThrustPlugin::~ThrustPlugin() {}

    void ThrustPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading the thrust_plugin");
        model_ = _parent;
        world_ = model_->GetWorld();


        node_namespace_ = "";
        cmd_timeout_ = 1.0; // how long to allow no input on cmd_drive

        param_mapping_type_ = 0;
        param_max_cmd_ = 1.0;
        param_max_force_fwd_ = 100.0;
        param_max_force_rev_ = -100.0;
        param_boat_width_ = 1.0;
        param_boat_length_ = 1.35;
        param_thrust_z_offset_ = -0.01;

        // prev_update_time_ = last_cmd_drive_time_ = this->world_->GetSimTime();


        // Initialize the ROS node and subscribe to cmd_drive
        // int argc = 0;
        // char** argv = NULL;

        // ros::init(argc, argv, "thrust_plugin", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        // rosnode_ = new ros::NodeHandle(node_namespace_);
        // cmd_drive_sub_ = rosnode_->subscribe("cmd_drive", 1, &UsvThrust::OnCmdDrive, this);

    }





    GZ_REGISTER_MODEL_PLUGIN(ThrustPlugin);
}
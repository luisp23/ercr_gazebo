#include <thrust_plugin.h>


namespace gazebo {

    ThrustPlugin::~ThrustPlugin() {}

    void ThrustPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading the thrust_plugin");
        model_ = _parent;
        world_ = model_->GetWorld();

        node_namespace_ = "";
        cmd_timeout_ = 1.0;
        param_mapping_type_ = 0;
        param_max_cmd_ = 1.0;
        param_max_force_fwd_ = 100.0;
        param_max_force_rev_ = -100.0;
        param_boat_width_ = 1.0;
        param_boat_length_ = 1.35;
        param_thrust_z_offset_ = -0.01;


        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(node_namespace_);

        cmd_drive_sub_ = node_handle_->Subscribe("cmd_drive", &ThrustPlugin::OnCmdDrive, this);


    }

    void ThrustPlugin::OnCmdDrive(ThrustPtr &thrust){

        // last_cmd_drive_time_ = this->world_->GetSimTime();
        last_cmd_drive_left_ = thrust->left();
        last_cmd_drive_right_ = thrust->right();

        std::cout << last_cmd_drive_left_ << std::endl; 
        std::cout << last_cmd_drive_right_ << std::endl; 
        
    }



    GZ_REGISTER_MODEL_PLUGIN(ThrustPlugin);
}
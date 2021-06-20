#include <thrust_plugin.h>


namespace gazebo {

    ThrustPlugin::~ThrustPlugin() {}

    void ThrustPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading the thrust_plugin");
        model_ = _parent;
        world_ = model_->GetWorld();

        node_namespace_ = "";

        // Load plugin parameters
        if (_sdf->HasElement("mappingType"))
            param_mapping_type_ = _sdf->GetElement("mappingType")->Get<int>();
        else
            gzerr << "[thrust_plugin] Please specify a mapping type.\n";

        if (_sdf->HasElement("boatWidth"))
            param_boat_width_ = _sdf->GetElement("boatWidth")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the boat width.\n";


        if (_sdf->HasElement("boatLength"))
            param_boat_length_ = _sdf->GetElement("boatLength")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the boat length.\n";

        if (_sdf->HasElement("thrustOffsetZ"))
            param_thrust_z_offset_ = _sdf->GetElement("thrustOffsetZ")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the thrust offset.\n";
        
        if (_sdf->HasElement("maxCmd"))
            param_max_cmd_ = _sdf->GetElement("maxCmd")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the maximum thrust command.\n";
        
        if (_sdf->HasElement("maxForceFwd"))
            param_max_force_fwd_ = _sdf->GetElement("maxForceFwd")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the maximum forward force.\n";

        if (_sdf->HasElement("maxForceRev"))
            param_max_force_rev_ = _sdf->GetElement("maxForceRev")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the maximum reverse force.\n";

        if (_sdf->HasElement("cmdTimeout"))
            cmd_timeout_ = _sdf->GetElement("cmdTimeout")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the command timeout.\n";


        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(node_namespace_);

        cmd_drive_sub_ = node_handle_->Subscribe("cmd_drive", &ThrustPlugin::OnCmdDrive, this);

        //  Enumerating model
        ROS_INFO_STREAM("Enumerating Model...");
        ROS_INFO_STREAM("Model name = "<< model_->GetName());

        physics::Link_V links = model_->GetLinks();
        for (unsigned int i = 0; i < links.size(); i++){
            ROS_INFO_STREAM("Link: "<< links[i]->GetName());
        }

        // Applying force to the base link
        link_ = model_->GetLink(links[0]->GetName());
        
        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ThrustPlugin::OnUpdate, this));

    }


    double ThrustPlugin::scaleThrustCmd(double cmd)
    {
        double val = 0.0;

        if (cmd >= 0.0)
        {
            val = cmd/param_max_cmd_*param_max_force_fwd_;
            val = std::min(val,param_max_force_fwd_);
        }else{
            // cmd is less than zero
            val = -1.0*std::abs(cmd)/param_max_cmd_*std::abs(param_max_force_rev_);
            val = std::max(val,param_max_force_rev_);
        }

        return val;  
    }


    double ThrustPlugin::glf(double x, float A, float K, float B, float v, float C, float M)
    {
        return A + (K-A) / (pow(C + exp(-B*(x - M)), 1.0/v));
    }

    double ThrustPlugin::glfThrustCmd(double cmd)
    {
        double val = 0.0;
        if (cmd > 0.01)
        {
            val = glf(cmd,0.01,59.82,5.0,0.38,0.56,0.28);
            val = std::min(val,param_max_force_fwd_);

        }else if (cmd < 0.01){ 
            val = glf(cmd,-199.13,-0.09,8.84,5.34,0.99,-0.57);
            val = std::max(val,param_max_force_rev_);
            
        }else{
            val = 0.0;
        }

        return val;
    }


    void ThrustPlugin::OnCmdDrive(ThrustPtr &thrust)
    {
        last_cmd_drive_time_ = world_->SimTime();
        
        last_cmd_drive_left_ = thrust->left();
        last_cmd_drive_right_ = thrust->right();
    }


    void ThrustPlugin::OnUpdate()
    {        
        common::Time time_now = world_->SimTime();
        prev_update_time_ = time_now;

        // Enforce command timeout
        double dcmd = (time_now - last_cmd_drive_time_).Double();
        if ( (dcmd > cmd_timeout_) && (cmd_timeout_ > 0.0))
        {
            // ROS_INFO_STREAM_THROTTLE(1.0,"Command timeout!");
            // last_cmd_drive_left_ = 0.0;
            // last_cmd_drive_right_ = 0.0;
        }

        double thrust_left = 0.0;
        double thrust_right = 0.0;

        switch(param_mapping_type_)
        {
            case 0: // Simplest, linear
                thrust_left = scaleThrustCmd(last_cmd_drive_left_);
                thrust_right = scaleThrustCmd(last_cmd_drive_right_);
                break;
            case 1: // GLF
                thrust_left = glfThrustCmd(last_cmd_drive_left_);
                thrust_right = glfThrustCmd(last_cmd_drive_right_);
                break;
            default:
                ROS_FATAL_STREAM("Cannot use mappingType=" << param_mapping_type_);
                break;
        } 

        double thrust = thrust_right + thrust_left;
        double torque = (thrust_right - thrust_left)*param_boat_width_;

        // Add torque
        link_->AddRelativeTorque(ignition::math::Vector3d(0,0,torque));

        // Add input force with offset below vessel and relative pos of thrusters
        ignition::math::Vector3d relpos(-1.0*param_boat_length_/2.0, 0.0 , param_thrust_z_offset_);  
        ignition::math::Vector3d inputforce3(thrust, 0,0);
        
        pose_ = link_->WorldPose();
        inputforce3 = pose_.Rot().RotateVector(inputforce3);
        
        link_->AddForceAtRelativePosition(inputforce3,relpos);
    }

    GZ_REGISTER_MODEL_PLUGIN(ThrustPlugin);
}
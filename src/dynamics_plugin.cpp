#include <dynamics_plugin.h>

#define GRAVITY 9.815

namespace gazebo {

    DynamicsPlugin::~DynamicsPlugin() {
        rosnode_->shutdown();
        spinner_thread_->join();
        delete rosnode_;
        delete spinner_thread_;
    }

    void DynamicsPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading the dynamics plugin...");
        model_ = _parent;
        world_ = model_->GetWorld();

        node_namespace_ = "";

        
        // Load plugin parameters
        if (_sdf->HasElement("waterLevel"))
            water_level_ = _sdf->GetElement("waterLevel")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the waterLevel parameter.\n";

        if (_sdf->HasElement("waterDensity"))
            water_density_ = _sdf->GetElement("waterDensity")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the waterDensity parameter.\n";

        if (_sdf->HasElement("xDotU"))
            param_X_dot_u_ = _sdf->GetElement("xDotU")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the yDotV parameter.\n";

        if (_sdf->HasElement("yDotV"))
            param_Y_dot_v_ = _sdf->GetElement("yDotV")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the yDotV parameter.\n";
        
        if (_sdf->HasElement("nDotR"))
            param_N_dot_r_ = _sdf->GetElement("nDotR")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the nDotR parameter.\n";
        
        if (_sdf->HasElement("xU"))
            param_X_u_ = _sdf->GetElement("xU")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the xU parameter.\n";

        if (_sdf->HasElement("xUU"))
            param_X_uu_ = _sdf->GetElement("xUU")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the xUU parameter.\n";

        if (_sdf->HasElement("yV"))
            param_Y_v_ = _sdf->GetElement("yV")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the yV parameter .\n";

        if (_sdf->HasElement("yVV"))
            param_Y_vv_ = _sdf->GetElement("yVV")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the yVV parameter.\n";
        
        if (_sdf->HasElement("zW"))
            param_Z_w_ = _sdf->GetElement("zW")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the zW parameter.\n";
        
        if (_sdf->HasElement("kP"))
            param_K_p_ = _sdf->GetElement("kP")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the kP parameter.\n";

        if (_sdf->HasElement("mQ"))
            param_M_q_ = _sdf->GetElement("mQ")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the mQ parameter.\n";

        if (_sdf->HasElement("nR"))
            param_N_r_ = _sdf->GetElement("nR")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the nR parameter.\n";

        if (_sdf->HasElement("nRR"))
            param_N_rr_ = _sdf->GetElement("nRR")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the nRR parameter.\n";

        if (_sdf->HasElement("boatWidth"))
            param_boat_width_ = _sdf->GetElement("boatWidth")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the boatWidth parameter.\n";

        if (_sdf->HasElement("boatLength"))
            param_boat_length_ = _sdf->GetElement("boatLength")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the boatLength parameter.\n";

        if (_sdf->HasElement("metacentricLength"))
            param_metacentric_length_ = _sdf->GetElement("metacentricLength")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the metacentricLength parameter.\n";

        if (_sdf->HasElement("metacentricWidth"))
            param_metacentric_width_ = _sdf->GetElement("metacentricWidth")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the metacentricWidth parameter.\n";

        if (_sdf->HasElement("boatArea"))
            param_boat_area_ = _sdf->GetElement("boatArea")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the boatArea parameter.\n";

        //  Enumerating model
        ROS_INFO_STREAM("Enumerating Model...");
        ROS_INFO_STREAM("Model name = "<< model_->GetName());
        physics::Link_V links = model_->GetLinks();

        for (unsigned int i = 0; i < links.size(); i++)
        {
            ROS_INFO_STREAM("Link: "<< links[i]->GetName());
        }

        // Applying force to the base link
        link_ = model_->GetLink(links[0]->GetName());
        
        // Wave parameters
        std::ostringstream buf;
        ignition::math::Vector2d tmpm;
        std::vector<float> tmpv(2,0);
        param_wave_n_ = _sdf->GetElement("wave_n")->Get<int>();

        for(int i=0; i < param_wave_n_; i++)
        {
            buf.str("");
            buf << "wave_amp" << i;
            param_wave_amps_.push_back(_sdf->GetElement(buf.str())->Get<float>());
            ROS_INFO_STREAM("Wave Amplitude " << i <<": " << param_wave_amps_[i]);
            buf.str("");
            buf << "wave_period" << i;
            param_wave_periods_.push_back(_sdf->GetElement(buf.str())->Get<float>());
            buf.str("");
            buf << "wave_direction" << i;
            tmpm = _sdf->GetElement(buf.str())->Get<ignition::math::Vector2d>();
            tmpv[0] = tmpm.X();
            tmpv[1] = tmpm.Y();
            param_wave_directions_.push_back(tmpv);
            ROS_INFO_STREAM("Wave Direction " << i << ": " << param_wave_directions_[i][0] << ", " << param_wave_directions_[i][1]);
        }

        // Get inertia and mass of vessel
        ignition::math::Vector3d inertia = link_->GetInertial()->PrincipalMoments();
        double mass = link_->GetInertial()->Mass();

        // Report some of the pertinent parameters for verification
        ROS_INFO("USV Dynamics Parameters: From URDF XACRO model definition");
        ROS_INFO_STREAM("Vessel Mass (rigid-body): " << mass);
        ROS_INFO_STREAM("Vessel Inertia Vector (rigid-body): X:" << inertia[0] << " Y:" << inertia[1] << " Z:"<<inertia[2]);

        prev_update_time_ = world_->SimTime();

        

        // Initialize the ROS node
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "dynamics_gazebo", 
        ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        rosnode_ = new ros::NodeHandle(node_namespace_);

        // Listen to the update event. This event is broadcast every simulation iteration.
		spinner_thread_ = new boost::thread(boost::bind( &DynamicsPlugin::spin, this));
		updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DynamicsPlugin::OnUpdate, this));


        // Initialize Added Mass Matrix
        Ma_ = Eigen::MatrixXd(6,6);
        Ma_ << param_X_dot_u_ ,   0,   0, 0, 0, 0,
            0,   param_Y_dot_v_,   0, 0, 0, 0,
            0,   0,   0.1, 0, 0, 0,
            0,   0,   0, 0.1, 0, 0, 
            0,   0,   0, 0, 0.1, 0,  
            0,   0,   0, 0, 0, param_N_dot_r_ ;

        Cmat_ =  Eigen::MatrixXd::Zero(6,6);
        Dmat_ =  Eigen::MatrixXd::Zero(6,6);
        state_dot_ = Eigen::VectorXd(6);
        state_ = Eigen::VectorXd(6);
        amassVec_ = Eigen::VectorXd(6);

        int NN = 2; // must be factor of 2! - only 2 for now!!
        
        // x,y grid step increments
        dx_ = param_boat_length_/NN;
        dy_ = param_boat_width_/NN;
        
        // Vector for interating throug grid points on boat
        for (int ii=-NN/2; ii<0; ii++)
        {
            II_.push_back(ii);
        }
        
        for (int ii=1; ii<=NN/2; ii++)
        {
            II_.push_back(ii);
        }
        
        // Precalculate this to save some time.
        buoy_frac_ = (param_boat_area_/(NN*NN))*GRAVITY*water_density_;

    }


    void DynamicsPlugin::spin()
    {
        while(ros::ok()) ros::spinOnce();
    }


    void DynamicsPlugin::OnUpdate()
    {        
        common::Time time_now = world_->SimTime();
        
        double dt = (time_now - prev_update_time_).Double();
        prev_update_time_ = time_now;

        // Get Pose/Orientation from Gazebo 
        pose_ = link_->WorldPose();
        euler_ = pose_.Rot().Euler();

        // Get body-centered linear and angular rates
        vel_linear_body_ = link_->RelativeLinearVel();  
        ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel linear: " << vel_linear_body_);

        vel_angular_body_ = link_->RelativeAngularVel();
        ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel angular: " << vel_angular_body_);
        
        // Estimate the linear and angular accelerations.
        // Note the the GetRelativeLinearAccel() and AngularAccel() functions
        // appear to be unreliable
        ignition::math::Vector3d accel_linear_body = (vel_linear_body_ - prev_lin_vel_) /dt;
        prev_lin_vel_ = vel_linear_body_;
        ROS_DEBUG_STREAM_THROTTLE(0.5,"Accel linear: " << accel_linear_body);

        ignition::math::Vector3d accel_angular_body = (vel_angular_body_ - prev_ang_vel_) /dt;
        prev_ang_vel_ = vel_angular_body_;
        ROS_DEBUG_STREAM_THROTTLE(0.5,"Accel angular: " << accel_angular_body);

        // Create state and derivative of state (accelerations)
        state_dot_ << accel_linear_body.X(), accel_linear_body.Y(), accel_linear_body.Z(), accel_angular_body.X(), accel_angular_body.Y(), accel_angular_body.Z();
        state_ << vel_linear_body_.X(), vel_linear_body_.Y(), vel_linear_body_.Z(), vel_angular_body_.X(), vel_angular_body_.Y(), vel_angular_body_.Z();

        // Added Mass
        amassVec_ = -1.0*Ma_*state_dot_;
        ROS_DEBUG_STREAM_THROTTLE(1.0,"state_dot_: \n" << state_dot_);
        ROS_DEBUG_STREAM_THROTTLE(1.0,"amassVec :\n" << amassVec_);
        
        // Coriolis - added mass components
        Cmat_(0,5) = param_Y_dot_v_ * vel_linear_body_.Y();
        Cmat_(1,5) = param_X_dot_u_ * vel_linear_body_.X();
        Cmat_(5,0) = param_Y_dot_v_ * vel_linear_body_.Y();
        Cmat_(5,1) = param_X_dot_u_ * vel_linear_body_.X();
        Cvec_ = -1.0*Cmat_*state_;
        ROS_DEBUG_STREAM_THROTTLE(1.0,"Cvec :\n" << Cvec_);
        
        // Drag
        //Eigen::MatrixXd Dmat = Eigen::MatrixXd(6,6);
        Dmat_(0,0) = param_X_u_ + param_X_uu_*std::abs(vel_linear_body_.X());
        Dmat_(1,1) = param_Y_v_ + param_Y_vv_*std::abs(vel_linear_body_.Y());
        Dmat_(2,2) = param_Z_w_;
        Dmat_(3,3) = param_K_p_;
        Dmat_(4,4) = param_M_q_;
        Dmat_(5,5) = param_N_r_ + param_N_rr_*std::abs(vel_angular_body_.Z());
        ROS_DEBUG_STREAM_THROTTLE(1.0,"Dmat :\n" << Dmat_);
        Dvec_ = -1.0*Dmat_*state_;
        ROS_DEBUG_STREAM_THROTTLE(1.0,"Dvec :\n" << Dvec_);

        // Vehicle frame transform
        tf2::Quaternion vq = tf2::Quaternion();
        tf2::Matrix3x3 m;
        m.setEulerYPR(euler_.Z(), euler_.Y(), euler_.X());
        m.getRotation(vq);
        tf2::Transform xform_v = tf2::Transform(vq);


        // Sum all forces - in body frame
        Eigen::VectorXd forceSum = amassVec_ + Dvec_; // + buoyVec;
        // Forces in fixed frame
        ROS_DEBUG_STREAM_THROTTLE(1.0,"forceSum :\n" << forceSum);
        
        // Add dynamic forces/torques to link at CG
        link_->AddRelativeForce(ignition::math::Vector3d(forceSum(0),forceSum(1),forceSum(2)));
        link_->AddRelativeTorque(ignition::math::Vector3d(forceSum(3),forceSum(4),forceSum(5)));

        // Distribute upward buoyancy forces
        float ddx, ddy, ddz, buoy_force;
        double w, k, dz, Ddotx;
        ignition::math::Vector3d X;     // location of vehicle base link
        tf2::Vector3 bpnt(0,0,0);       // grid points on boat
        tf2::Vector3 bpnt_w(0,0,0);     // in world coordinates


        // Loop over boat grid points
        for (std::vector<int>::iterator it = II_.begin(); it != II_.end(); ++it)
        {
            bpnt.setX((*it)*dx_);  // grid point in boat fram
            for (std::vector<int>::iterator jt = II_.begin(); jt != II_.end(); ++jt)
            {
                bpnt.setY((*jt)*dy_);
      
                // Transform from vessel to water/world frame
                bpnt_w = xform_v*bpnt;

                // Debug
                ROS_DEBUG_STREAM_THROTTLE(1.0, "[" << (*it) << "," << (*jt) << "] grid points" << bpnt.x() <<"," << bpnt.y() << "," << bpnt.z());
                ROS_DEBUG_STREAM_THROTTLE(1.0,"v frame euler " << euler_);
                ROS_DEBUG_STREAM_THROTTLE(1.0,"in water frame" << bpnt_w.x() << "," << bpnt_w.y() <<"," << bpnt_w.z());

                // Vertical location of boat grid point in world frame
                ddz = pose_.Pos().Z() + bpnt_w.z();
                
                ROS_DEBUG_STREAM("Z, pose: " << pose_.Pos().Z() << ", bpnt: " << bpnt_w.z() << ", dd: " << ddz);
                
                // Find vertical displacement of wave field
                X.X() = pose_.Pos().X() + bpnt_w.x();  // World location of grid point
                X.Y() = pose_.Pos().Y() + bpnt_w.y();
                
                // sum vertical dsplacement over all waves
                dz = 0.0;
                for (int ii=0; ii < param_wave_n_; ii++)
                {
                    Ddotx = param_wave_directions_[ii][0]*X.X() +param_wave_directions_[ii][1]*X.Y();
                    w = 2.0*3.14159 / param_wave_periods_[ii];
                    k = w*w/9.81;
                    dz += param_wave_amps_[ii]*cos(k*Ddotx-w*time_now.Float());;	
                }
                
                ROS_DEBUG_STREAM_THROTTLE(1.0, "wave disp: " << dz);

                // Buoyancy force at grid point
                buoy_force = (((water_level_ + dz) - (ddz))*(buoy_frac_));
                ROS_DEBUG_STREAM("buoy_force: " << buoy_force);
                
                // Apply force at grid point
                // From web, Appears that position is in the link frame and force is in world frame
                link_->AddForceAtRelativePosition(ignition::math::Vector3d(0, 0, buoy_force), ignition::math::Vector3d(bpnt.x(),bpnt.y(),bpnt.z()));
            }
        }

    }

    GZ_REGISTER_MODEL_PLUGIN(DynamicsPlugin);
}
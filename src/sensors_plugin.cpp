#include <sensors_plugin.h>


namespace gazebo {

    SensorsPlugin::~SensorsPlugin() {}

    void SensorsPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading the sensors_plugin");
        model_ = _parent;
        world_ = model_->GetWorld();

        node_namespace_ = "";
        bool _imu_initialized = false; 
        // Declare the sensor manager
        sensors::SensorManager *pMgr = sensors::SensorManager::Instance() ;
        
        // Used to get the scoped names of the sensors
        // TODO: Use this to automatically get the scoped sensor names without hardcoding it
        // for (sensors::SensorPtr sensor : sensors::SensorManager::Instance()->GetSensors())
        //     gzmsg << sensor->ScopedName() << "\n";

        if(pMgr == nullptr){
            gzerr << "[sensors_plugin] Could not initialize the sensor manager!.\n";

        }else{
            
            // Get the IMU sensor
            pGetImuSensor_ = pMgr->GetSensor("water_world::M200::M200::imu_link::imu");
            if(pGetImuSensor_ == nullptr){
                gzerr << "[sensors_plugin] Could not find the IMU sensor!.\n";
            }else{
                
                pImuSensor_ = std::dynamic_pointer_cast<sensors::ImuSensor, sensors::Sensor>(pGetImuSensor_);
                if(pImuSensor_ == nullptr){
                    gzerr << "[sensors_plugin] Could not initialize the IMU sensor!.\n";
                }else{ 
                    _imu_initialized = true; 
                }
            }

            // TODO: Add get the GPS plugin





        }


        // Transport node for gazebo transport library topics
        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(node_namespace_);

        // ROS node to subscribe/publish to ROS topics
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "thrust_plugin", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        rosnode_ = new ros::NodeHandle(node_namespace_);

        ros_imu_pub_ = rosnode_->advertise<sensor_msgs::Imu>("/imu", 1);


        if(_imu_initialized)
            updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SensorsPlugin::OnUpdate, this));
    }



    void SensorsPlugin::OnUpdate()
    {        
        ignition::math::Quaterniond _orientation =	pImuSensor_->Orientation(); 
        ignition::math::Vector3d _angularVelocity = pImuSensor_->AngularVelocity();
        ignition::math::Vector3d _linearAcceleration = pImuSensor_->LinearAcceleration();


        // TODO: Figure out how to extract/computer the covariance matrices associated with this plugin
        imu_msg_.header.stamp = ros::Time::now(); 
        
        imu_msg_.orientation.x = _orientation.X(); 
        imu_msg_.orientation.y = _orientation.Y();
        imu_msg_.orientation.z = _orientation.Z();
        imu_msg_.orientation.w = _orientation.W();

        imu_msg_.angular_velocity.x = _angularVelocity.X(); 
        imu_msg_.angular_velocity.y = _angularVelocity.Y();
        imu_msg_.angular_velocity.z = _angularVelocity.Z();

        imu_msg_.linear_acceleration.x = _linearAcceleration.X(); 
        imu_msg_.linear_acceleration.y = _linearAcceleration.Y();
        imu_msg_.linear_acceleration.z = _linearAcceleration.Z();

        ros_imu_pub_.publish(imu_msg_);

    }

    GZ_REGISTER_MODEL_PLUGIN(SensorsPlugin);
}
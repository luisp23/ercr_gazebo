#include <sensors_plugin.h>

namespace gazebo {

    SensorsPlugin::~SensorsPlugin() {}

    void SensorsPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading the sensors_plugin");
        model_ = _parent;
        world_ = model_->GetWorld();

        node_namespace_ = "";

        // Load plugin parameters
        if (_sdf->HasElement("imuPubRate"))
            imu_pub_rate_ = _sdf->GetElement("imuPubRate")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the IMU sensor publishing rate.\n";

        if (_sdf->HasElement("gpsPubRate"))
            gps_pub_rate_ = _sdf->GetElement("gpsPubRate")->Get<double>();
        else
            gzerr << "[thrust_plugin] Please specify the GPS sensor publishing rate.\n";


        bool _imu_initialized = false; 
        bool _gps_initialized = false; 

        // Declare the sensor manager
        sensors::SensorManager *pMgr = sensors::SensorManager::Instance() ;
        
        // Used to get the scoped names of the sensors
        // TODO: Use this to automatically get the scoped sensor names without hardcoding it
        for (sensors::SensorPtr sensor : sensors::SensorManager::Instance()->GetSensors())
            gzmsg << sensor->ScopedName() << "\n";

        if(pMgr == nullptr){
            gzerr << "[sensors_plugin] Could not initialize the sensor manager!.\n";

        }else{
            
            // Get the IMU sensor
            pGetImuSensor_ = pMgr->GetSensor("water_world::M200::M200::imu_link::imu_sensor");
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

            pGetGpsSensor_ = pMgr->GetSensor("water_world::M200::M200::gps_link::gps_sensor");
            if(pGetGpsSensor_ == nullptr){
                gzerr << "[sensors_plugin] Could not find the GPS sensor!.\n";
            }else{
                
                pGpsSensor_ = std::dynamic_pointer_cast<sensors::GpsSensor, sensors::Sensor>(pGetGpsSensor_);
                if(pGpsSensor_ == nullptr){
                    gzerr << "[sensors_plugin] Could not initialize the GPS sensor!.\n";
                }else{ 
                    _gps_initialized = true; 
                }
            }


            // TODO: Add magnetometer sensor
        

        }


        // Transport node for gazebo transport library topics
        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(node_namespace_);

        // ROS node to subscribe/publish to ROS topics
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "thrust_plugin", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        rosnode_ = new ros::NodeHandle(node_namespace_);
        
        imu_timer_ = rosnode_->createTimer(ros::Duration(1/imu_pub_rate_), &SensorsPlugin::imuPublishCallback, this);
        gps_timer_ = rosnode_->createTimer(ros::Duration(1/gps_pub_rate_), &SensorsPlugin::gpsPublishCallback, this);

        ros_imu_pub_ = rosnode_->advertise<sensor_msgs::Imu>("imu/data", 1);
        ros_gps_pub_ = rosnode_->advertise<sensor_msgs::NavSatFix>("gps/fix", 1);

        // Not needed
        // updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SensorsPlugin::OnUpdate, this));
    
    }

    void SensorsPlugin::OnUpdate()
    {   
        // Not needed
    }

    void SensorsPlugin::imuPublishCallback(const ros::TimerEvent &event){

        // Get IMU sensor data 
        ignition::math::Quaterniond _imu_orientation =	pImuSensor_->Orientation(); 
        ignition::math::Vector3d _imu_angularVelocity = pImuSensor_->AngularVelocity();
        ignition::math::Vector3d _imu_linearAcceleration = pImuSensor_->LinearAcceleration();
        
        // TODO: Figure out how to extract/computer the covariance matrices associated with these plugins

        // Setup the IMU ROS message 
        imu_msg_.orientation.x = _imu_orientation.X(); 
        imu_msg_.orientation.y = _imu_orientation.Y();
        imu_msg_.orientation.z = _imu_orientation.Z();
        imu_msg_.orientation.w = _imu_orientation.W();
        imu_msg_.orientation_covariance = {0,0,0,0,0,0,0,0,0};

        imu_msg_.angular_velocity.x = _imu_angularVelocity.X(); 
        imu_msg_.angular_velocity.y = _imu_angularVelocity.Y();
        imu_msg_.angular_velocity.z = _imu_angularVelocity.Z();
        imu_msg_.angular_velocity_covariance = {0,0,0,0,0,0,0,0,0};

        imu_msg_.linear_acceleration.x = _imu_linearAcceleration.X(); 
        imu_msg_.linear_acceleration.y = _imu_linearAcceleration.Y();
        imu_msg_.linear_acceleration.z = _imu_linearAcceleration.Z();
        imu_msg_.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};

        // Publish the messages
        imu_msg_.header.stamp = ros::Time::now();
        imu_msg_.header.frame_id = "imu_link";
        
        ros_imu_pub_.publish(imu_msg_);
    }

    void SensorsPlugin::gpsPublishCallback(const ros::TimerEvent &event){
        
        // Get GPS sensor data 
        ignition::math::Angle _gps_latitude = pGpsSensor_->Latitude(); 
        ignition::math::Angle _gps_longitude = pGpsSensor_->Longitude (); 
        double _gps_altitude = pGpsSensor_->Altitude (); 

        // Setup the GPS ROS message 
        gps_msg_.latitude = _gps_latitude.Degree();
        gps_msg_.longitude = _gps_longitude.Degree(); 
        gps_msg_.altitude = _gps_altitude;
        gps_msg_.position_covariance = {0,0,0,0,0,0,0,0,0};
        gps_msg_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;   

        gps_msg_.header.stamp = ros::Time::now();
        gps_msg_.header.frame_id = "gps_link"; 

        ros_gps_pub_.publish(gps_msg_);
    }




    GZ_REGISTER_MODEL_PLUGIN(SensorsPlugin);
}

#ifndef THRUST_PLUGIN_H
#define THRUST_PLUGIN_H

#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>

#include "gazebo/transport/transport.hh"
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <memory>
#include <iostream>


#include <gazebo/sensors/sensors.hh>
#include "sensor_msgs/Imu.h"

namespace gazebo
{

	


	class SensorsPlugin : public ModelPlugin
  	{

    	public:
			SensorsPlugin(){
                
			}

        virtual ~SensorsPlugin();

		protected: 
			virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
			virtual void OnUpdate();
	
    	private: 

			physics::ModelPtr model_;
			physics::WorldPtr world_;
			physics::LinkPtr link_;
		
			std::string node_namespace_;
			std::string link_name_;

			
            sensors::SensorPtr pGetImuSensor_; 
            sensors::ImuSensorPtr pImuSensor_;
             
            
            
            
            
            
            transport::NodePtr node_handle_;
			transport::SubscriberPtr cmd_drive_sub_;
			
			ros::NodeHandle *rosnode_;
            ros::Publisher ros_imu_pub_; 
            
            sensor_msgs::Imu imu_msg_; 

			event::ConnectionPtr updateConnection_;
  	}; 
}



#endif // THRUST_PLUGIN_H

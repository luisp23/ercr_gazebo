
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
#include "sensor_msgs/NavSatFix.h"


namespace gazebo
{
	static constexpr double defaultImuPubRate = 1000.0; 
	static constexpr double defaultGpsPubRate = 10.0;
	


	class SensorsPlugin : public ModelPlugin
  	{
    	public:
			SensorsPlugin():
				imu_pub_rate_(defaultImuPubRate),
				gps_pub_rate_(defaultGpsPubRate){
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


			double imu_pub_rate_, gps_pub_rate_;

            sensors::SensorPtr pGetImuSensor_; 
            sensors::ImuSensorPtr pImuSensor_;

			sensors::SensorPtr pGetGpsSensor_; 
            sensors::GpsSensorPtr pGpsSensor_;
             
            transport::NodePtr node_handle_;
			transport::SubscriberPtr cmd_drive_sub_;
			
			ros::NodeHandle *rosnode_;

			ros::Timer imu_timer_;
			ros::Timer gps_timer_;

            ros::Publisher ros_imu_pub_;
			ros::Publisher ros_gps_pub_;

            sensor_msgs::Imu imu_msg_;
			sensor_msgs::NavSatFix gps_msg_;  

			void imuPublishCallback(const ros::TimerEvent &event);
			void gpsPublishCallback(const ros::TimerEvent &event);

			event::ConnectionPtr updateConnection_;
		
  	}; 
}



#endif // THRUST_PLUGIN_H

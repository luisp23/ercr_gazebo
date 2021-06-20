
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

#include <ercr_msgs/Thrust.h>
#include "Thrust.pb.h"




// #include <gazebo/gazebo_client.hh>

namespace gazebo
{

	typedef const boost::shared_ptr<const ercr_msgs::msgs::Thrust> ThrustPtr;
	
	static constexpr int defaultMapping = 0;
	static constexpr double defaultBoatWidth = 1.0; 
	static constexpr double defaultBoatLength = 1.35; 
	static constexpr double defaultThrustOffset = -0.01; 
	static constexpr double defaultMaxCmd = 1.0;
	static constexpr double defaultMaxForceFwd = 100.0;
	static constexpr double defaultMaxForceRev = -100.0;
	static constexpr double defaultCmdTimeout = 1.0; 

	class ThrustPlugin : public ModelPlugin
  	{

    	public:
			ThrustPlugin():
				param_mapping_type_(defaultMapping),
				param_boat_width_(defaultBoatWidth), 
				param_boat_length_(defaultBoatLength), 
				param_thrust_z_offset_(defaultThrustOffset), 
				param_max_cmd_(defaultMaxCmd), 
				param_max_force_fwd_(defaultMaxForceFwd), 
				param_max_force_rev_(defaultMaxForceRev), 
				cmd_timeout_(defaultCmdTimeout){
			}

        virtual ~ThrustPlugin();

		protected: 
			virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
			virtual void OnUpdate();
	
    	private: 

			physics::ModelPtr model_;
			physics::WorldPtr world_;
			physics::LinkPtr link_;
			
			common::Time prev_update_time_;
			common::Time last_cmd_drive_time_;  

			double last_cmd_drive_left_;
			double last_cmd_drive_right_;

			int param_mapping_type_;
			double param_boat_width_;
			double param_boat_length_;
			double param_thrust_z_offset_;

			double param_max_cmd_;
			double param_max_force_fwd_;
			double param_max_force_rev_;
			double cmd_timeout_;
			
			std::string node_namespace_;
			std::string link_name_;

			transport::NodePtr node_handle_;
			transport::SubscriberPtr cmd_drive_sub_;
			
			ros::NodeHandle *rosnode_;
    		ros::Subscriber cmd_drive_sub_ros;

			ignition::math::Pose3d pose_; 

			double scaleThrustCmd(double cmd);
			double glf(double x, float A, float K, float B, float v, float C, float M);
			double glfThrustCmd(double cmd);
			void OnCmdDrive(ThrustPtr &thrust);
			void OnCmdDriveRos(const ercr_msgs::ThrustConstPtr &thrust);
			event::ConnectionPtr updateConnection_;
  	}; 
}



#endif // THRUST_PLUGIN_H

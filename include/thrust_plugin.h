
#ifndef THRUST_PLUGIN_H
#define THRUST_PLUGIN_H

#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>
#include <ercr_msgs/Thrust.h>

#include <ros/ros.h>
// #include <ros/callback_queue.h>
// #include <ros/advertise_options.h>

namespace gazebo
{
  class ThrustPlugin : public ModelPlugin
  {

    public:
    	ThrustPlugin(){}

        virtual ~ThrustPlugin();



	
	protected: 
		virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	


    private: 

		physics::ModelPtr model_;
		physics::WorldPtr world_;
		physics::LinkPtr link_;


		double cmd_timeout_;
		common::Time prev_update_time_;
		common::Time last_cmd_drive_time_;  

		double last_cmd_drive_left_;
		double last_cmd_drive_right_;

		int param_mapping_type_;

		/*! Plugin Parameter: Maximum (abs val) of Drive commands. typ. +/-1.0 */
		double param_max_cmd_;

		/*! Plugin Parameter: Maximum forward force [N] */
		double param_max_force_fwd_;

		/*! Plugin Parameter: Maximum reverse force [N] */
		double param_max_force_rev_;

		/*! Plugin Parameter: Boat width [m] */
		double param_boat_width_;

		/*! Plugin Parameter: Boat length [m] */
		double param_boat_length_;

		/*! Plugin Parameter: Z offset for applying forward thrust */
		double param_thrust_z_offset_;
		

		std::string node_namespace_;
		std::string link_name_;

		// ros::NodeHandle *rosnode_;
		// ros::Subscriber cmd_drive_sub_;

		// event::ConnectionPtr updateConnection;
		    
  }; 
}



#endif // THRUST_PLUGIN_H

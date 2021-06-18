
#ifndef THRUST_PLUGIN_H
#define THRUST_PLUGIN_H

#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>
#include <ercr_msgs/Thrust.h>


#include "gazebo/transport/transport.hh"
#include <ros/ros.h>


#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "Thrust.pb.h"




// #include <gazebo/gazebo_client.hh>

namespace gazebo
{

	typedef const boost::shared_ptr<const ercr_msgs::msgs::Thrust> ThrustPtr;

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
			double param_max_cmd_;
			double param_max_force_fwd_;
			double param_max_force_rev_;
			double param_boat_width_;
			double param_boat_length_;
			double param_thrust_z_offset_;
			
			std::string node_namespace_;

			std::string link_name_;

			transport::NodePtr node_handle_;
			transport::SubscriberPtr cmd_drive_sub_;

			void OnCmdDrive(ThrustPtr &thrust);


		// event::ConnectionPtr updateConnection;
		    
  	}; 
}



#endif // THRUST_PLUGIN_H

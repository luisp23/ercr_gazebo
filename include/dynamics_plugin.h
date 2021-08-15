
#ifndef DYNAMICS_PLUGIN_H
#define DYNAMICS_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math.hh>

#include <Eigen/Core>

#include <ignition/math/Vector3.hh>

#include "gazebo/transport/transport.hh"
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>


// #include <gazebo/gazebo_client.hh>

namespace gazebo
{
	static constexpr double defaultWaterLevel = 0.5;
	static constexpr double defaultWaterDensity = 997.7735;

	static constexpr double defaultxDotU = 5.0;
	static constexpr double defaultyDotV = 5.0;
	static constexpr double defaultnDotR = 1.0; 

	static constexpr double defaultxU = 20.0;
	static constexpr double defaultxUU = 0.0;
	static constexpr double defaultyV = 20.0;
	static constexpr double defaultyVV = 0.0;
	static constexpr double defaultzW = 20.0;
	static constexpr double defaultkP = 20.0;
	static constexpr double defaultmQ = 20.0;
	static constexpr double defaultnR = 20.0;
	static constexpr double defaultnRR = 0.0;
	
	static constexpr double defaultmetacentricLength = 0.0;	
	static constexpr double defaultmetacentricWidth = 20.0;

	static constexpr double defaultboatLength = 0.0;	
	static constexpr double defaultboatWidth = 20.0;
	static constexpr double defaultboatArea = 0.48;

	class DynamicsPlugin : public ModelPlugin
  	{

    	public:
			DynamicsPlugin():
				water_level_(defaultWaterLevel),
				water_density_(defaultWaterDensity),
				param_X_dot_u_(defaultxDotU),
				param_Y_dot_v_(defaultyDotV),
				param_N_dot_r_(defaultnDotR),
				param_X_u_(defaultxU),
				param_X_uu_(defaultxUU),
				param_Y_v_(defaultyV),
				param_Y_vv_(defaultyVV),
				param_Z_w_(defaultzW),
				param_K_p_(defaultkP),
				param_M_q_(defaultmQ),
				param_N_r_(defaultnR),
				param_N_rr_(defaultnRR),
				param_metacentric_length_(defaultmetacentricLength),
				param_metacentric_width_(defaultmetacentricWidth),
				param_boat_width_(defaultboatLength),
				param_boat_length_(defaultboatWidth),
				param_boat_area_(defaultboatArea){
			}

        virtual ~DynamicsPlugin();

		protected: 
			virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
			virtual void OnUpdate();

	
    	private: 

			physics::ModelPtr model_;
			physics::WorldPtr world_;
			physics::LinkPtr link_;

			std::string link_name_;

			std::string node_namespace_;

			/*! Plugin Parameter: Added mass in surge, X_\dot{u} */
			double param_X_dot_u_;
			/*! Plugin Parameter: Added mass in sway, Y_\dot{v} */
			double param_Y_dot_v_;
			/*! Plugin Parameter: Added mass in yaw, N_\dot{r}*/
			double param_N_dot_r_;

			/*! Plugin Parameter: Linear drag in surge */
			double param_X_u_;
			/*! Plugin Parameter: Quadratic drag in surge */
			double param_X_uu_;
			/*! Plugin Parameter: Linear drag in sway */
			double param_Y_v_;
			/*! Plugin Parameter: Quadratic drag in sway */
			double param_Y_vv_;
			
			double param_Z_w_;
			double param_K_p_;
			double param_M_q_;

			/*! Plugin Parameter: Linear drag in yaw */
			double param_N_r_;
			/*! Plugin Parameter: Quadratic drag in yaw*/
			double param_N_rr_;
			/*! Plugin Parameter: Boat width [m] */
			double param_boat_width_;
			/*! Plugin Parameter: Boat length [m] */
			double param_boat_length_;
			/*! Plugin Parameter: Horizontal surface area [m^2] */
			double param_boat_area_ ;
			/*! Plugin Parameter: Metacentric length [m] */
			double param_metacentric_length_;
			/*! Plugin Parameter: Metacentric width[m] */
			double param_metacentric_width_;

			double xyz_damping_;
			double yaw_damping_;
			double rp_damping_;

			/* Water height [m]*/
			double water_level_;
			/* Water density [kg/m^3] */
			double water_density_;

			common::Time prev_update_time_;

			/*! Added mass matrix, 6x6 */
			Eigen::MatrixXd Ma_;

			ignition::math::Vector3d prev_lin_vel_;
			ignition::math::Vector3d prev_ang_vel_;
			ignition::math::Pose3d pose_;
			ignition::math::Vector3d euler_;
			ignition::math::Vector3d vel_linear_body_;
			ignition::math::Vector3d vel_angular_body_;
			ignition::math::Vector3d acceleration;
			ignition::math::Vector3d angular_velocity_;
			ignition::math::Vector3d angular_acceleration_;
			
			Eigen::VectorXd state_dot_;
			Eigen::VectorXd state_;
			Eigen::VectorXd amassVec_;
			Eigen::MatrixXd Cmat_;
			Eigen::VectorXd Cvec_;
			Eigen::MatrixXd Dmat_;
			Eigen::VectorXd Dvec_;

			// For Buoyancy calculation
			float buoy_frac_;
			float dx_;
			float dy_;
			std::vector<int> II_;

			/* Wave parameters */
			int param_wave_n_;
			std::vector<float> param_wave_amps_;
			std::vector<float> param_wave_periods_;
			std::vector< std::vector<float> > param_wave_directions_;


			ros::NodeHandle *rosnode_;
			boost::thread *spinner_thread_;




    		void spin(); 




			// Pointer to the update event connection
			event::ConnectionPtr updateConnection_;

  	}; 
}



#endif // DYNAMICS_PLUGIN_H

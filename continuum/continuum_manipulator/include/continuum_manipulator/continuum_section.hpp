#ifndef CONTINUUM_SECTION_HPP
#define CONTINUUM_SECTION_HPP

#include <iostream>
#include <math.h>
#include <algorithm>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "continuum_msgs/msg/cable_states.hpp"
#include "continuum_msgs/msg/config_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace continuum_manipulator
{
	class ContinuumSection: public rclcpp::Node
	{
	public:
		ContinuumSection();
		ContinuumSection(int num_sec_);
		~ContinuumSection(void);	

		/// Parâmetros construtivos da seção
		int num_sec_ = 0; 		// Número da seção no robô
		int num_joints_;		// Número de juntas físicas da seção
		int num_cables_; 		// Número total de cabos do robô
		int num_sec_cables_; 	// Número de cabos da seção
		int num_joints_urdf_; 	// Número de juntas do URDF da seção
		Eigen::VectorXd a_;		// Raio dos cabos
		double b_; 				// Comprimento dos semi-elos na seção
		Eigen::VectorXd alpha_; // Ângulo de outras seções com relação a essa
		double preload_ = 0; 	// Pré-tensão dos cabos na seção
		void readParameters();

		// int init_urdf_jnts_=-1; // Junta a partir da qual inicia a seção no URDF
		/// Espaço dos cabos
		KDL::JntArrayVel cablePosVel_; // Contém comprimento TOTAL dos cabos.
		KDL::JntArray cableEffort_;

		void setCables(
			const Eigen::VectorXd& cablepos,
			const Eigen::VectorXd& cablevel,
			const Eigen::VectorXd& tensions){
				Eigen::VectorXd::Map(&cablePosVel_.q.data[0],cablepos.rows())=cablepos;
				Eigen::VectorXd::Map(&cablePosVel_.qdot.data[0],cablevel.rows())=cablevel;
				Eigen::VectorXd::Map(&cableEffort_.data[0],tensions.rows())=tensions;
			}

		Eigen::Vector4d getDistalCables()const{ // Primeiros 4 cabos
			return cablePosVel_.q.data(Eigen::seq(0,3));};
		Eigen::VectorXd getProximalCables()const{ // Últimos cabos
			return cablePosVel_.q.data(Eigen::lastN(num_cables_-4));};
		Eigen::Vector4d getSectionCables(int sec)const{
			return cablePosVel_.q.data(Eigen::seqN((sec-1)*4,4));};

		/// Espaço de configuração - U, V e h (Allen 2020)
		Eigen::Vector3d configPos_ = Eigen::Vector3d{0,0,0};  
		Eigen::Vector3d configVel_ = Eigen::Vector3d{0,0,0};

		//Espaço de juntas (Jones 2010)
		KDL::JntArrayVel jointPosVel_;
		KDL::JntArray jointEffort_; //TODO: Passar isso para um wrench
		
		double theta_ = 0; 				// Flexão (ângulo TOTAL de uma seção)
		double theta_d_ = 0;			// Velocidade angular de flexão
		double phi_ = 0;				// Azimute 
		double phi_d_ = 0;				// Velocidade angular do azimute
		double kappa_ = 0;				// Curvatura
		double kappa_d_ = 0;			// Taxa de variação da curvatura

		//Jacobianos
		Eigen::MatrixXd jacCableCfg_(const Eigen::VectorXd &cables){
			return jacCableJnt_(std::move(cables));
				// * jacJntCfg_(fkSpecific(std::move(cables)));
		};
		Eigen::MatrixXd jacCfgCable_(const Eigen::Vector3d &cfg){
			return jacJntCable_(screw2jnt(cfg));
				// * jacCfgJnt_(cfg);
		};

		// Jacobiano de configuração para ângulos theta e phi
		Eigen::MatrixXd jacCfgAJnt_(const Eigen::Vector3d &cfg); //Allen
		Eigen::MatrixXd jacJntCfgA_(const Eigen::Vector2d &jnts);  //Allen

		Eigen::MatrixXd jacJntCable_(const Eigen::Vector2d &jnts);
		Eigen::MatrixXd jacCableJnt_(const Eigen::VectorXd &cables){
			return jacJntCable_(screw2jnt(fkSpecific(cables))).
				completeOrthogonalDecomposition().pseudoInverse();
		};

		//Mapeamentos de força

		Eigen::MatrixXd forceJacCableJnt_(const Eigen::Vector3d &cfg);
		Eigen::MatrixXd forceJacJntCable_(const Eigen::Vector3d &cfg){
			RCLCPP_INFO_STREAM(get_logger(),"Called forceJacJntCable_...");
			Eigen::MatrixXd result = forceJacCableJnt_(std::move(cfg)).
				completeOrthogonalDecomposition().pseudoInverse();
			RCLCPP_INFO_STREAM(get_logger(),"got the pinv...");
			return result;
		};

		// Conversão entre parametrizações
		Eigen::Vector3d jnt2screw(const Eigen::Vector2d &jnt_params);
		Eigen::Vector2d screw2jnt(const Eigen::Vector3d &screw_params);

		/// Mapeamentos específicos da seção - implementados pelas classes derivadas
		Eigen::Isometry3d fkIndependent(const Eigen::Vector3d& cfg);
		Eigen::Isometry3d fkIndependent_u(const Eigen::Vector3d& cfg){
			return fkIndependent(Eigen::Vector3d(cfg)/num_joints_);
		};
		Eigen::VectorXd ikSpecific(
			const Eigen::Vector3d& cfg,
			const Eigen::VectorXd& a,
			const Eigen::VectorXd& alpha
		);
		Eigen::Vector3d fkSpecific(const Eigen::VectorXd &jointvalues);
		// Mapeia do espaço de "juntas" (cabos) pra espaço de config

		// ROS
		// rclcpp::TimerBase::SharedPtr timer_;
		// double time_step_ = 1e-2;

		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
		rclcpp::Publisher<continuum_msgs::msg::CableStates>::SharedPtr cableStatePub_;
		rclcpp::Publisher<continuum_msgs::msg::ConfigStates>::SharedPtr configStatePub_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr configVizPub_;
		std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> cableVizPub_;

		void jointStateCB(const sensor_msgs::msg::JointState::SharedPtr jntmsg);
		void cablesCB(const continuum_msgs::msg::CableStates::SharedPtr cablemsg);
		void timerCB();

		visualization_msgs::msg::MarkerArray cableMarkerArray_;

		rclcpp::TimerBase::SharedPtr timer_;
		double t0_;
	};
}
#endif
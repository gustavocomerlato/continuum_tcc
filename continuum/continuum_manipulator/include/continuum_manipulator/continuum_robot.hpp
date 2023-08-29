#ifndef CONTINUUM_ROBOT_HPP
#define CONTINUUM_ROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>
#include <numeric>

namespace continuum_manipulator
{
	class ContinuumRobot: public rclcpp::Node
	{
	protected:
		/// Parâmetros construtivos do robô
		int num_sections_ = 0;					// Número de seções
		std::vector<int> num_joints_; 			// Número de juntas em cada seção
		std::vector<double> b_;					// Comprimentos de semi-elos em cada seção
        Eigen::VectorXd alphas_;         		// Ângulo de rotação em Z entre seções
		KDL::Frame refFrm_;						// Frame de referência mandado pelo gerador de trajetórias
		void readParameters();

		//Cadeia cinemática da parte serial do manipulador
		std::string robotDescription_;
		KDL::Tree tree_;
		KDL::Chain chain_;
		KDL::ChainFkSolverPos *fkSolverPos_;
		KDL::ChainFkSolverVel *fkSolverVel_;
		KDL::ChainIkSolverVel *ikSolverVel_;

		KDL::JntArrayVel jointPosVel_;
		KDL::JntArray 	 jointEffort_;

		rclcpp::TimerBase::SharedPtr timer_;
		double t0_;

		//Solver de cinemática inversa
		// Achei mais fácil de vincular ao código do robô em si, ao invés de ficar preso à estrutura do KDL
		int maxiter_=500;
		double eps_pos_=1e-5;
		double eps_ang_=1e-3;	
		int ikSolverPos_setup(
			int maxiter=500,
			double eps_pos=1e-3,
			double eps_ang=1e-3
		);

		int ikSolverPos_FABRIKc(
			const KDL::JntArray& q_init,
			const KDL::Frame& T_base_goal,
			KDL::JntArray& q_out
		);

		Eigen::MatrixXd calcConfigJacobian_pos();

		std::string ikSolverPos_errorStr(const int error)const;

		std::vector<Eigen::Vector3d> P_; //Posição dos pontos de início das seções, usado no solver de IK FABRIK e FABRIKc
		std::vector<Eigen::Vector3d> Pmid_; // Posição dos pontos de início das seções, usado no solver de IK FABRIKc
		std::vector<Eigen::Vector3d> Zb_;	// Vetores normais à seção transversal do início das seções
		std::vector<Eigen::Vector3d> Ze_;	// Vetores normais à seção transversal do final das seções
		std::vector<double> lt_; //Metade do comprimento tangente da seção, usado no solver de IK
		std::vector<int> thetaJnts; // tamanho num_sec_
		std::vector<int> secJnts; // tamanho num_sec_
		std::vector<int> phiJnts; // tamanho num_sec_
		std::vector<double> L0_; 	// comprimentos iniciais das seções
		Eigen::Vector3d Z_goal;
		Eigen::Vector3d H;
		Eigen::Matrix<double,6,1> delta_pos;
		KDL::Frame T_base_head;
		bool logging = false;
		bool ready = false;
		bool running = false;
	public:
		static const int E_NO_ERROR = 0;
		static const int E_MAX_ITERATIONS_EXCEEDED = -100;
    	static const int E_INCREMENT_JOINTS_TOO_SMALL = -101;
		static const int E_SIZE_MISMATCH = -102;

		ContinuumRobot();
		~ContinuumRobot(void);

        // void addSection(std::unique_ptr<ContinuumSection> section, double angle);
		//Publishers/Subscribers
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub_;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trajPointSub_;
		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr trajTwistSub_;
		
		void jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates);
		void trajPointCB(const geometry_msgs::msg::PoseStamped::SharedPtr cartpnt);
		void trajTwistCB(const geometry_msgs::msg::TwistStamped::SharedPtr cartvel);
		void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDesc);
		void timerCB() const;
	};
}
#endif

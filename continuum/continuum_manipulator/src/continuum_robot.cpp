#include "continuum_manipulator/continuum_robot.hpp"
// #include "chainiksolverpos_continuumfabrik.cpp"

namespace KDL{

template<typename Derived>
inline void Twist_to_Eigen(const KDL::Twist& t,Eigen::MatrixBase<Derived>& e){
    e(0)=t.vel.data[0];
	e(1)=t.vel.data[1];
	e(2)=t.vel.data[2];
	e(3)=t.rot.data[0];
	e(4)=t.rot.data[1];
	e(5)=t.rot.data[2];
}

template<typename Derived>
inline void Frame_to_Eigen(const KDL::Frame& t,Eigen::MatrixBase<Derived>& e){
	e(0,0) = t.M(0,0);
	e(0,1) = t.M(0,1);
	e(0,2) = t.M(0,2);
	e(1,0) = t.M(1,0);
	e(1,1) = t.M(1,1);
	e(1,2) = t.M(1,2);
	e(2,0) = t.M(2,0);
	e(2,1) = t.M(2,1);
	e(2,2) = t.M(2,2);
	e(0,3) = t.p(0);
	e(1,3) = t.p(1);
	e(2,3) = t.p(2);
	e(3,3) = 1;
}

template<typename Derived>
inline void Vector_to_Eigen(const KDL::Vector& t, Eigen::MatrixBase<Derived>& e){
    e(0)=t(0);
    e(1)=t(1);
    e(2)=t(2);
}

template<typename Derived>
inline void Eigen_to_Vector(const Eigen::MatrixBase<Derived>& t,KDL::Vector& e){
    e(0)=t(0);
    e(1)=t(1);
    e(2)=t(2);
}
}

namespace continuum_manipulator
{
	/**
	 * @brief Inicializa o robô.
	 * 
	 */
	ContinuumRobot::ContinuumRobot(): Node("continuum_manipulator"){
		using std::placeholders::_1; 
		rclcpp::QoS qos(rclcpp::KeepLast(1));
		qos.transient_local();
		
		trajPointSub_=create_subscription<geometry_msgs::msg::PoseStamped>(
			"/traj_pose",100,std::bind(
				&ContinuumRobot::trajPointCB,this,std::placeholders::_1));
		trajTwistSub_=create_subscription<geometry_msgs::msg::TwistStamped>(
			"/traj_twist",100,std::bind(
				&ContinuumRobot::trajTwistCB,this,std::placeholders::_1));
		jointStatePub_=create_publisher<sensor_msgs::msg::JointState>(
			"/joint_states",100);

		//Waits for robot description to be available
		auto robotDescriptionSubscriber_=
			create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&ContinuumRobot::robotDescriptionCB,this,std::placeholders::_1));

		while(robotDescription_.empty())
		{
			RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
			rclcpp::spin_some(get_node_base_interface());
		}

		KDL::Tree tree;
		if (!kdl_parser::treeFromString(robotDescription_,tree))
			RCLCPP_ERROR_STREAM(get_logger(),"Failed to construct KDL tree.");
		
		// for(int ii=0;ii<tree.getNrOfSegments();ii++)
		// 	RCLCPP_INFO_STREAM(get_logger(),"Segment " << ii << ": " << tree.);

		if (!tree.getChain("originLink","endLink",chain_))
			RCLCPP_ERROR_STREAM(get_logger(),"Failed to get chain from KDL tree.");

		t0_=now().seconds();
		using namespace std::chrono_literals;
		timer_=rclcpp::create_timer(this,this->get_clock(),50ms,std::bind(&ContinuumRobot::timerCB,this));
		jointPosVel_.resize(chain_.getNrOfJoints()); SetToZero(jointPosVel_);

		readParameters();
		// Solvers de cinemática
		fkSolverPos_=new KDL::ChainFkSolverPos_recursive(chain_);
		fkSolverVel_=new KDL::ChainFkSolverVel_recursive(chain_);
		// if(num_sections_>1){
			ikSolverPos_setup(10);
			// ikSolverPos_=new KDL::ChainIkSolverPos_LMA(chain_,L);
			// ikSolverPos_=new KDL::ChainIkSolverPos_ContinuumFabrik(chain_,L);
		// }
		ikSolverVel_=new KDL::ChainIkSolverVel_pinv(chain_); //Pseudo-inversa direta
		// ikSolverVel_=new KDL::ChainIkSolverVel_pinv_givens(chain_); //Pseudo-inversa direta
		// ikSolverVel_=new KDL::ChainIkSolverVel_wdls(chain_); // Pseudo-inversa com minimos quadrados amortecidos
		// ikSolverVel_ = new KDL::ChainIkSolverVel_pinv_nso(chain_);
		ready=true;
	}

	/**
	 * @brief Destroi uma seção por método default.
	 * 
	 */
	ContinuumRobot::~ContinuumRobot(){
		delete fkSolverPos_;
		delete fkSolverVel_;
		// delete ikSolverPos_;
		delete ikSolverVel_;
	}

	/**
	 * @brief Lê parâmetros do robô. Configura a, b, alpha, num_cables_, preload_, num_joints_
	 * 
	 */
	void ContinuumRobot::readParameters(void)
	{
		declare_parameter("num_sec",rclcpp::PARAMETER_INTEGER);
		if(!get_parameter("num_sec",num_sections_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'num_sec' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		if(logging) RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'num_sec' as " << num_sections_);
		b_.resize(num_sections_); L0_.resize(num_sections_); num_joints_.resize(num_sections_);

		//Parâmetros para FABRIKc
		phiJnts.resize(num_sections_);	thetaJnts.resize(num_sections_); secJnts.resize(num_sections_+1);
		lt_.resize(num_sections_); Pmid_.resize(num_sections_);  P_.resize(num_sections_+1);
		Zb_.resize(num_sections_); Ze_.resize(num_sections_);

		std::vector<double> tmp;
		declare_parameter("alpha",rclcpp::PARAMETER_DOUBLE_ARRAY);
		if(!get_parameter("alpha",tmp))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'alpha' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		alphas_ = Eigen::Map<Eigen::VectorXd>(tmp.data(),tmp.size());
		if(logging) RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'alpha' as " << alphas_);
		
		std::vector<long int> tmp2;
		declare_parameter("num_joints",rclcpp::PARAMETER_INTEGER_ARRAY);
		if(!get_parameter("num_joints",tmp2))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'num_joints' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		for (int ii=0;ii<int(tmp2.size());ii++)
			num_joints_[ii] = int(tmp2[ii]);
		if(logging) RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'num_joints'");
		
		declare_parameter("b",rclcpp::PARAMETER_DOUBLE_ARRAY);
		if(!get_parameter("b",b_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'b' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		if(logging) RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'b'");

		//Alguns outros parâmetros importantes
		for (int ii=0;ii<num_sections_;ii++){
			L0_[ii] = 2 * num_joints_[ii] * b_[ii];
		}

		//Identificando quais frames são de inicio e de fim de seção
		int activecnt=0; secJnts[num_sections_]=chain_.getNrOfSegments();
		for (int jnt=0;jnt<int(chain_.getNrOfSegments());jnt++){
			std::string name = chain_.getSegment(jnt).getJoint().getName();
			int sec = name.at(1) - '0'; // seção onde está a junta
			if(chain_.getSegment(jnt).getJoint().getType()!=KDL::Joint::JointType::None&&
				chain_.getSegment(jnt).getJoint().getType()!=KDL::Joint::JointType::Fixed){

				if (name.size()==7 && name.at(3)=='T') phiJnts[sec] =activecnt; // Junta mestre do ângulo phi
				else if(name.at(3)=='0'){thetaJnts[sec] = activecnt; secJnts[sec] = jnt;}
				activecnt++;
			}
		}
	}

	void ContinuumRobot::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
	{
		robotDescription_=robotDescription->data;
	}

	/**
	 * @brief Recebe um ponto cartesiano, transforma em valores de juntas.
	 * 
	 * @param cartpnt PoseStamped com orientação e posição gerado por um gerador de trajetórias
	 */
	void ContinuumRobot::trajPointCB(const geometry_msgs::msg::PoseStamped::SharedPtr cartpnt){
		// checar se já alocamos toda a memória
		if(!ready) return;
		if(logging) RCLCPP_INFO_STREAM(get_logger(),"Received trajectory point!");
		KDL::JntArray q_out=jointPosVel_.q;

		refFrm_.p[0] = cartpnt->pose.position.x;
     	refFrm_.p[1] = cartpnt->pose.position.y;
     	refFrm_.p[2] = cartpnt->pose.position.z;
     	refFrm_.M = KDL::Rotation::Quaternion( cartpnt->pose.orientation.x,
			cartpnt->pose.orientation.y,
			cartpnt->pose.orientation.z,
			cartpnt->pose.orientation.w);
		int error=0;
		if(num_sections_>1 && !running){
			if((error=ikSolverPos_FABRIKc(jointPosVel_.q,refFrm_,q_out))<0)
				RCLCPP_ERROR_STREAM(get_logger(),"Failed to compute inverse kinematics: (" << error << ") "
					<< ikSolverPos_errorStr(error));
		}else if(num_sections_==1){
			double phi, theta;
			phi=atan2(refFrm_.p[1],refFrm_.p[0]);
			if(refFrm_.p[2]>0){
				theta=acos(1-2*(pow(refFrm_.p[0],2)+pow(refFrm_.p[1],2))/(pow(refFrm_.p[0],2)+pow(refFrm_.p[1],2)+pow(refFrm_.p[2],2)));
			}else{
				theta=2*M_PI - acos(1-2*(pow(refFrm_.p[0],2)+pow(refFrm_.p[1],2))/(pow(refFrm_.p[0],2)+pow(refFrm_.p[1],2)+pow(refFrm_.p[2],2)));
			}
			theta=theta/num_joints_[0];
			jointPosVel_.q.data[0] = phi;
			jointPosVel_.q.data[num_joints_[0]+1] = -phi;
			jointPosVel_.q.data.segment(1,num_joints_[0]).setConstant(theta);
			return;
		}
		// if((error=ikSolverPos_->CartToJnt(jointPosVel_.q,refFrm_,q_out)) < 0)
		// 	RCLCPP_ERROR_STREAM(get_logger(),"Failed to compute inverse kinematics: (" << error << ") "
		// 		<< ikSolverPos_->strError(error));
		jointPosVel_.q=q_out;
	}

	/**
	 * @brief Recebe uma velocidade cartesiana, transforma em velocidades de juntas.
	 * 
	 * @param cartpnt TwistStamped com velocidades lineares e RPY
	 */
	void ContinuumRobot::trajTwistCB(const geometry_msgs::msg::TwistStamped::SharedPtr cartvel){
		if(!ready) return;
		if(logging) RCLCPP_INFO_STREAM(get_logger(),"Received trajectory twist!");
		KDL::JntArray q_out=jointPosVel_.qdot;
		const KDL::Twist twist(
			KDL::Vector(
				cartvel->twist.linear.x,
				cartvel->twist.linear.y,
				cartvel->twist.linear.z),
			KDL::Vector(
				cartvel->twist.angular.x,
				cartvel->twist.angular.y,
				cartvel->twist.angular.z));
		// twist.vel[0] = cartvel->twist.linear.x;
     	// twist.vel[1] = cartvel->twist.linear.y;
     	// twist.vel[2] = cartvel->twist.linear.z;
		// twist.rot[0] = cartvel->twist.angular.x;
     	// twist.rot[1] = cartvel->twist.angular.y;
     	// twist.rot[2] = cartvel->twist.angular.z;
		int error=0;
		if(num_sections_>1){
			if((error=ikSolverVel_->CartToJnt(jointPosVel_.qdot,twist,q_out)) < 0) //Chamada via pseudo-inversa
				RCLCPP_ERROR_STREAM(get_logger(),"Failed to compute inverse velocity kinematics: (" << error << ") "
					<< ikSolverVel_->strError(error));
		}else{
			//Cinemática diferencial inversa caseira para velocidades das juntas de uma seção

			Eigen::MatrixXd jZX = calcConfigJacobian_pos();
			const auto &xd = twist.vel.x();
			const auto &yd = twist.vel.y();
			const auto &zd = twist.vel.z();
			Eigen::Vector3d xdv; xdv << xd, yd, zd;
			Eigen::Vector2d qvels;
			qvels = jZX*xdv;
			q_out.data[0] = qvels[0];
			q_out.data[num_joints_[0]+1] = -qvels[0];
			q_out.data.segment(1,num_joints_[0]).setConstant(qvels[1]);
		}
		// if((error=ikSolverVel_->CartToJnt(jointPosVel_.qdot,KDL::FrameVel(chain_.getSegment(num_joints_[0]).getFrameToTip(),twist),	q_out)) < 0)
		// 	RCLCPP_ERROR_STREAM(get_logger(),"Failed to compute inverse velocity kinematics: (" << error << ") "
		// 		<< ikSolverVel_->strError(error));
		jointPosVel_.qdot=q_out;
	}


	// Jacobiano entre velocidades cartesianas e e velocidades de configuração no espaço (Phi, Theta)
	Eigen::MatrixXd ContinuumRobot::calcConfigJacobian_pos(){
		Eigen::MatrixXd jacZetaX(2,3);
		const auto &x = refFrm_.p[0];
		const auto &y = refFrm_.p[1];
		const auto &z = refFrm_.p[2];
		double xy2 = pow(x,2)+pow(y,2);
		jacZetaX << -y/xy2, x/xy2, 0,
					2*x*z/sqrt(xy2)/(xy2+pow(z,2)), 2*y*z/sqrt(xy2)/(xy2+pow(z,2)), -2*sqrt(xy2)/(xy2+pow(z,2));
		return jacZetaX;
	}

	void ContinuumRobot::timerCB(void) const
	{
		sensor_msgs::msg::JointState jsmsg;
		jsmsg.header.frame_id = "originLink";
		jsmsg.header.stamp=now();

		jsmsg.name.resize(jointPosVel_.q.data.size());
		jsmsg.position.resize(jointPosVel_.q.data.size());
		jsmsg.velocity.resize(jointPosVel_.qdot.data.size());
		//Nomeia as juntas
		int jnt=0;
		for (int ii=0;ii<int(chain_.getNrOfSegments());ii++){
			if(chain_.getSegment(ii).getJoint().getType()!=KDL::Joint::JointType::Fixed &&
				chain_.getSegment(ii).getJoint().getType()!=KDL::Joint::JointType::None){
				jsmsg.name[jnt] = chain_.getSegment(ii).getJoint().getName(); jnt++;
			}
		}

		Eigen::VectorXd::Map(&jsmsg.position[0], jointPosVel_.q.data.size()) = jointPosVel_.q.data;
		Eigen::VectorXd::Map(&jsmsg.velocity[0], jointPosVel_.qdot.data.size()) = jointPosVel_.qdot.data;
		jointStatePub_->publish(jsmsg);
	}

	int ContinuumRobot::ikSolverPos_setup(int maxiter, double eps_pos, double eps_ang){
		if(maxiter<0||eps_pos<0||eps_ang<0) return -1;
		maxiter_=maxiter;
		eps_pos_=eps_pos;
		eps_ang_=eps_ang;
		return 1;
	}

	std::string ContinuumRobot::ikSolverPos_errorStr(const int error)const{
		if(E_MAX_ITERATIONS_EXCEEDED == error) return "The maximum number of iterations was exceeded.";
		else if(E_INCREMENT_JOINTS_TOO_SMALL == error) return "The joint position increments are too small.";
		else if(E_SIZE_MISMATCH == error) return "There was a size mismatch with either the input or output joint arrays.";
		else return "No errors.";
	}

///****FABRIKC*****
	int ContinuumRobot::ikSolverPos_FABRIKc(const KDL::JntArray& q_init,const KDL::Frame& T_base_goal,KDL::JntArray& q_out){
		//Impementação do algoritmo FABRIKc, de Zhang, Yang, Dong e Xu (2018)
		running=true;
		if (chain_.getNrOfJoints() != q_init.rows() || chain_.getNrOfJoints() != q_out.rows()){
			running=false;
			return E_SIZE_MISMATCH;
		}

		double delta_pos_norm; double delta_ang_norm;
		KDL::JntArray q;
		q.resize(q_init.rows());
		q.data = q_init.data.cast<double>();

		double theta = 0.0;
		double phi 	 = 0.0;
		Eigen::Vector3d normal;
		KDL::Vector_to_Eigen(T_base_goal.M.UnitZ(),Z_goal);

		//Passo 1: Inicialização de P e lt de cada seção
		P_[0].setZero();
		for (int j=0;j<num_sections_;j++){
			fkSolverPos_->JntToCart(q,T_base_head,secJnts[j+1]);
			KDL::Vector_to_Eigen(T_base_head.p,P_[j+1]);		
			theta		= q(thetaJnts[j]);
			
			if (fabs(theta)<=1e-4) lt_[j]=L0_[j]/2;
			else lt_[j] = (b_[j] * tan(theta/2))/tan(theta/2/num_joints_[j]);
			KDL::Vector_to_Eigen(T_base_head.p - lt_[j]*T_base_head.M.UnitZ(),Pmid_[j]);
		}
		fkSolverPos_->JntToCart(q,T_base_head);		
		Twist_to_Eigen( KDL::diff( T_base_head, T_base_goal), delta_pos ); // Diferença entre frame final atual e objetivo armazenada como delta_pos
		delta_pos_norm = delta_pos.topRows(3).norm();
		delta_ang_norm = delta_pos.bottomRows(3).norm();

		if(logging) RCLCPP_INFO_STREAM(get_logger(),
				"------- iteration " << 0 << " ----------------\n"
			<< "  q               = " << q.data.transpose() << "\n"
			<< "  goal 	          = " << T_base_goal.p.x() << "   " << T_base_goal.p.y() << "   " << T_base_goal.p.z() << "\n"
			<< "  tool frame pos  = " << T_base_head.p.x() << "   " << T_base_head.p.y() << "   " << T_base_head.p.z() << "\n"
			<< "  difference      = "   << delta_pos.transpose() << "\n"
			<< "  pos diff norm   = "   << delta_pos_norm << "\n"
			<< "  ang diff norm   = "   << delta_ang_norm*180/M_PI << "\n" << std::endl);

		if (delta_pos_norm<eps_pos_ && delta_ang_norm <eps_ang_) { // Se já está dentro do permissível, não faz nada
			q_out.data      = q.data.cast<double>();
			running=false;
			return E_NO_ERROR;
		}
		
		int i=0; delta_pos_norm=fabs(eps_pos_)*10; 
		while(i<maxiter_ && delta_pos_norm>eps_pos_) { // Iterações do solver
			//******
			//Forward Reaching
			//  Posiciona a junta final no objetivo, e juntas intermediárias em retas que tentam chegar nos pontos originais
			//  A junta final é já posicionada com Z orientado de acordo com o frame objetivo.
			//	As juntas virtuais nos meios das seções ficam nos encontros dos eixos.
			//******
			T_base_head = T_base_goal;
			Vector_to_Eigen(T_base_goal.p,P_[num_sections_]);
			Ze_[num_sections_-1] = Z_goal;
			for(int j=num_sections_-1; j>=0;--j){
				if(logging) RCLCPP_INFO_STREAM(get_logger(),
            			"------- forward reaching, section " << j << " ----------------" << std::endl);
				// Chute inicial
				Pmid_[j]  = P_[j+1] - lt_[j]*Ze_[j];
				if (j+1==1) Zb_[j] << 0,0,1;
				else Zb_[j] = (Pmid_[j] - Pmid_[j-1])/(Pmid_[j] - Pmid_[j-1]).norm();
				// theta 		= acos(Zb_[j].dot(Ze_[j]));
				theta = atan2( (Zb_[j].cross(Ze_[j])).norm(),Zb_[j].dot(Ze_[j]));
				if(logging){ RCLCPP_INFO_STREAM(get_logger(),
							"chute inicial: \n"
						<< "  P("<<j+1<<")    = " << P_[j+1].transpose() <<"\n"
						<< "  P_mid("<<j<<")  = " << Pmid_[j].transpose() << std::endl);
					if (j>0) RCLCPP_INFO_STREAM(get_logger(),"  P_mid("<<j-1<<") = " << Pmid_[j-1].transpose() << std::endl);
					RCLCPP_INFO_STREAM(get_logger(),
						"  theta           = " << theta << "\n"
						<< "  Ze              = " << Ze_[j].transpose() << "\n"
						<< "  Zb              = " << Zb_[j].transpose() << "\n"
						<< "  lt              = " << lt_[j] << std::endl);
				}
				// Correção
				lt_[j]	= (fabs(theta)<=1e-6)? L0_[j]/2: (b_[j] * tan(theta/2))/tan(theta/2/num_joints_[j]); 
				Pmid_[j]	= P_[j+1] - lt_[j]*Ze_[j];
				if (j+1==1) Zb_[j] << 0,0,1;
				else Zb_[j] = (Pmid_[j] - Pmid_[j-1])/(Pmid_[j] - Pmid_[j-1]).norm();
				P_[j] 	= Pmid_[j] - Zb_[j] * lt_[j];
				theta 		= acos(Zb_[j].dot(Ze_[j]));
				if (j+1>1) Ze_[j-1] = Zb_[j];
				if(logging){RCLCPP_INFO_STREAM(get_logger(),
							"pós-correção: \n"
						<< "  P("<<j<<")      = " << P_[j].transpose() <<"\n"
						<< "  P_mid("<<j<<")  = " << Pmid_[j].transpose() << std::endl);
				if (j>0) RCLCPP_INFO_STREAM(get_logger(),"  P_mid("<<j-1<<") = " << Pmid_[j-1].transpose() << std::endl);
					RCLCPP_INFO_STREAM(get_logger(),
						"  theta           = " << theta  << "\n"
						<< "  Zb              = " << Zb_[j].transpose() << "\n"
						<< "  lt              = " << lt_[j] <<  std::endl);
				}
				
				if(j+1==num_sections_){
					for (int k=thetaJnts[j]; k<int(q.rows()-1);k++)
						q(k) = theta/num_joints_[j];
				}else{
					for (int k=thetaJnts[j]; k<phiJnts[j+1]-1;k++)
						q(k) = theta/num_joints_[j];
				}

			}
			if(logging) RCLCPP_INFO_STREAM(get_logger(),
            			"------- after forward reaching, iteration " << i << " ----------------\n"
                    << "  base pos  = " << P_[0].transpose() << "\n" << std::endl);

			//******
			//Backward Reaching 
			//  Posiciona a junta inicial na base, e repete o processo feito no forwards reaching, mas para cima
			//******
			P_[0].setZero(); Zb_[0] << 0, 0, 1;
			Pmid_[0] 	<< 0, 0, lt_[0];
			for(int j=0; j<num_sections_;++j){
				if(logging) RCLCPP_INFO_STREAM(get_logger(),
            			"------- backwards reaching, section " << j << " ----------------" << std::endl);
				// Chute
				Pmid_[j]	= P_[j] + lt_[j]*Zb_[j];
				if (j==num_sections_-1) Ze_[j] = Z_goal;
				else Ze_[j]		= (Pmid_[j+1] - Pmid_[j])/((Pmid_[j+1] - Pmid_[j]).norm());
				theta = atan2( (Zb_[j].cross(Ze_[j])).norm(),Zb_[j].dot(Ze_[j]));

				if(logging){RCLCPP_INFO_STREAM(get_logger(),
						"chute inicial: \n"
						<< "  P("<<j<<")      = " << P_[j].transpose()    << "\n"
						<< "  P_mid("<<j<<")  = " << Pmid_[j].transpose() << std::endl);
					if (j<num_sections_-1) RCLCPP_INFO_STREAM(get_logger(),"  P_mid("<<j+1<<") = " << Pmid_[j+1].transpose() << std::endl);
					RCLCPP_INFO_STREAM(get_logger(),
						"  theta           = " << theta  << "\n"
						<< "  Ze              = " << Ze_[j].transpose() << "\n"
						<< "  Zb              = " << Zb_[j].transpose() << "\n"
						<< "  lt              = " << lt_[j] << std::endl);
				}
				// Correção
				lt_[j]		= (fabs(theta)<=1e-6)? L0_[j]/2: (b_[j] * tan(theta/2))/tan(theta/2/num_joints_[j]);
				Pmid_[j]	= P_[j] + lt_[j]*Zb_[j];
				Ze_[j]		= (j==num_sections_-1)? Z_goal:((Pmid_[j+1] - Pmid_[j])/(Pmid_[j+1] - Pmid_[j]).norm());
				P_[j+1] 	= Pmid_[j] + Ze_[j] * lt_[j];
				theta = atan2( (Zb_[j].cross(Ze_[j])).norm(),Zb_[j].dot(Ze_[j]));
				if (j+1>1) Zb_[j+1] = Ze_[j];
				if(logging){RCLCPP_INFO_STREAM(get_logger(),
						"pós-correção: \n"
						<< "  P("<<j+1<<")    = " << P_[j+1].transpose() <<"\n"
						<< "  P_mid("<<j<<")  = " << Pmid_[j].transpose() << std::endl);
					if (j<num_sections_-1) RCLCPP_INFO_STREAM(get_logger(),"  P_mid("<<j+1<<") = " << Pmid_[j+1].transpose() << std::endl);
					RCLCPP_INFO_STREAM(get_logger(),
						"  theta           = " << theta  << "\n"
						<< "  Ze              = " << Ze_[j].transpose() << "\n"
						<< "  lt              = " << lt_[j] <<  std::endl);
				}
				
				//Atualização de juntas da seção
				if(j==num_sections_-1){
					q(phiJnts[j]) 		=  phi;
					q(int(q.rows()-1))	= -phi;
					for (int k=thetaJnts[j]; k<int(q.rows()-1);k++)
						q(k) = theta/double(int(q.rows()-1)-thetaJnts[j]);
				}else{
					q(phiJnts[j]) 		=  phi;
					q(phiJnts[j+1]-1) 	= -phi;
					for (int k=thetaJnts[j]; k<phiJnts[j+1]-1;k++)
						q(k) = theta/double(phiJnts[j+1]-1-thetaJnts[j]);
				}	
			}
			if(logging) RCLCPP_INFO_STREAM(get_logger(),
            			"------- after backward reaching, iteration " << i << " ----------------\n"
                    << "  q               = " << q.data.transpose() << std::endl);

			KDL::Vector_to_Eigen(T_base_goal.p,H);
			delta_pos_norm = (H - P_[num_sections_]).norm();
			i++;
		} // Iterações

		// Ajuste da orientação
		T_base_head = KDL::Frame::Identity();
		for (int ii=0;ii<num_sections_;ii++){
			KDL::Vector_to_Eigen(T_base_head.Inverse(KDL::Vector(P_[ii+1](0),P_[ii+1](1),P_[ii+1](2))),H);
			phi = atan2(H(1),H(0));
			if(logging) RCLCPP_INFO_STREAM(get_logger(),
							"section " << ii << ": \n"
							<< " P_["<<ii+1<<"] = " << P_[ii+1].transpose() << "\n"
							<< " H = " << H.transpose() << "\n"
							<< " phi = " << phi << "\n"
							<< " secJnts["<<ii+1<<"] = " << secJnts[ii+1] << std::endl);
			if (ii==num_sections_-1){
				q(phiJnts[ii]) 		=  phi;
				q(int(q.rows()-1))	= -phi;
			}else{
				q(phiJnts[ii]) 		=  phi;
				q(phiJnts[ii+1]-1) 	= -phi;
			}
			fkSolverPos_->JntToCart(q,T_base_head,secJnts[ii+1]);
		}

						
		fkSolverPos_->JntToCart(q,T_base_head);
		Twist_to_Eigen( diff( T_base_head, T_base_goal), delta_pos );
		delta_pos_norm = delta_pos.topRows(3).norm();
		delta_ang_norm = delta_pos.bottomRows(3).norm();

		KDL::Vector_to_Eigen(T_base_head.M.UnitZ(),H);
		q_out.data      = std::move(q.data.cast<double>());

		if(logging) RCLCPP_INFO_STREAM(get_logger(),
			"------- final joint states ----------------\n"
			<< "  q               = "   << q.data.transpose() << "\n"
			<< "  goal pos        = " << T_base_goal.p.x() << "   " << T_base_goal.p.y() << "   " << T_base_goal.p.z() << "\n"
			<< "  tool frame pos  = " << T_base_head.p.x() << "   " << T_base_head.p.y() << "   " << T_base_head.p.z() << "\n"
			<< "  difference      = "   << delta_pos.transpose() << "\n"
			<< "  pos diff norm   = "   << delta_pos_norm << "\n"
			<< "  goal Z axis     = "   << Z_goal.transpose() << "\n"
			<< "  tool Z axis     = "   << T_base_head.M.UnitZ().x() << "   " << T_base_head.M.UnitZ().y() << "   " << T_base_head.M.UnitZ().z() << "\n"
			<< "  Z diff norm     = "   << (Z_goal-H).norm() << "\n"
			<< "  iterations      = "   << i << std::endl);

		running=false;	
		return E_NO_ERROR;
	}
}


int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<continuum_manipulator::ContinuumRobot>());
	rclcpp::shutdown();
	return 0;
}
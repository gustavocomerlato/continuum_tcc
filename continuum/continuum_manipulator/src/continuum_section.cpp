#include <continuum_manipulator/continuum_section.hpp>

namespace continuum_manipulator
{
	using std::placeholders::_1;

	/**
	 * @brief Constroi uma seção por método default. Inútil?
	 * 
	 */
	ContinuumSection::ContinuumSection(): Node("continuum_section_1"){};

	/**
	 * @brief Construtor de seção contínua com elos rígidos e discos - Arquitetura similar ao Elephant Trunk
	 * 
	 * @param num_sec_ Número da seção (base = 0)
	 */
	ContinuumSection::ContinuumSection(int num_sec_):
			Node("continuum_section_"+std::to_string(num_sec_))
	{
		//ROS2
		rclcpp::QoS qos(rclcpp::KeepLast(1));
		qos.transient_local();

		jointStateSub_=create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states", 100,
			std::bind(&ContinuumSection::jointStateCB,this,std::placeholders::_1));
		cableStatePub_=create_publisher<continuum_msgs::msg::CableStates>(
			"/section_cable_states", 100);
		configStatePub_=create_publisher<continuum_msgs::msg::ConfigStates>(
			"/section_config_states", 100);
		configVizPub_=create_publisher<visualization_msgs::msg::Marker>(
			"/config_markers", 100);
		cableVizPub_=this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/cable_markers", rclcpp::QoS(1000));

		readParameters();

		//Setup
		jointPosVel_.resize(num_joints_urdf_); jointEffort_.resize(num_joints_urdf_);
		SetToZero(jointPosVel_); SetToZero(jointEffort_);

		cablePosVel_.resize(num_sec_cables_); cableEffort_.resize(num_sec_cables_);
		SetToZero(cablePosVel_); SetToZero(cableEffort_);
		
		auto L0 = 2 * b_; //Comprimentos iniciais dos cabos
		cablePosVel_.q.data = Eigen::VectorXd::Constant(num_sec_cables_,L0);
		cableEffort_.data = Eigen::VectorXd::Constant(num_sec_cables_,preload_);


		cableMarkerArray_.markers.resize(num_joints_*num_sec_cables_);
		t0_=now().seconds();
		using namespace std::chrono_literals;
		timer_=rclcpp::create_timer(this,this->get_clock(),50ms,std::bind(&ContinuumSection::timerCB,this));
	};

	/**
	 * @brief Destroi uma seção por método default.
	 * 
	 */
	ContinuumSection::~ContinuumSection(){
	};

	/**
	 * @brief Lê parâmetros do robô. Configura a, b, alpha, num_cables_, preload_, num_joints_
	 * 
	 */
	void ContinuumSection::readParameters(void)
	{
		declare_parameter("num_sec",rclcpp::PARAMETER_INTEGER);
		if(!get_parameter("num_sec",num_sec_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'num_sec' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'num_sec' as " << num_sec_);
		num_sec_-=1;

		declare_parameter("num_joints",rclcpp::PARAMETER_INTEGER);
		if(!get_parameter("num_joints",num_joints_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'num_joints' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'num_joints' as " << num_joints_);

		declare_parameter("num_cables",rclcpp::PARAMETER_INTEGER);
		if(!get_parameter("num_cables",num_cables_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'num_cables' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'num_cables' as " << num_cables_);
		
		declare_parameter("num_sec_cables",rclcpp::PARAMETER_INTEGER);
		if(!get_parameter("num_sec_cables",num_sec_cables_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'num_cables' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'num_sec_cables' as " << num_sec_cables_);

		declare_parameter("num_joints_urdf",rclcpp::PARAMETER_INTEGER);
		if(!get_parameter("num_joints_urdf",num_joints_urdf_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'num_joints_urdf' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'num_joints_urdf' as " << num_joints_urdf_);

		std::vector<double> tmp;
		a_.resize(num_sec_cables_/4);
		declare_parameter("a",rclcpp::PARAMETER_DOUBLE_ARRAY);
		if(!get_parameter("a",tmp))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'a' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		// a_=Eigen::VectorXd(tmp.data(),tmp.size());
		a_ = Eigen::Map<Eigen::VectorXd>(tmp.data(),tmp.size());
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'a' as " << a_);

		declare_parameter("b",rclcpp::PARAMETER_DOUBLE);
		if(!get_parameter("b",b_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'b' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'b' as " << b_);

		alpha_.resize(num_sec_cables_/4);
		declare_parameter("alpha",rclcpp::PARAMETER_DOUBLE_ARRAY); //declara o parâmetro que será lido do YAML de configuração
		if(!get_parameter("alpha",tmp))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'alpha' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		// alpha_=Eigen::VectorXd(tmp.data(),tmp.size());
		alpha_ = Eigen::Map<Eigen::VectorXd>(tmp.data(),tmp.size());
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'alpha' as " << alpha_);


		declare_parameter("preload",rclcpp::PARAMETER_DOUBLE);
		if(!get_parameter("preload",preload_))
		{
			RCLCPP_ERROR_STREAM(get_logger(),"No 'preload' parameter in node " << get_fully_qualified_name() << ".");
			return;
		}
		RCLCPP_INFO_STREAM(get_logger(),"Read parameter 'preload' as " << preload_);
	}

	/**
	 * @brief Processa JointStateMessage. Salva phi e theta. Calcula variáveis de configuração.
	 * 
	 * @param jntmsg 
	 */
	void ContinuumSection::jointStateCB(const sensor_msgs::msg::JointState::SharedPtr jntmsg)
	{
		//Encontra quais juntas da mensagem estamos interessados
		int ii=0;
		for(int cnt=0;cnt<int(jntmsg->name.size());cnt++){
			std::string name = jntmsg->name[cnt];
			int sec = name.at(1) - '0'; // seção onde está a junta
			if(sec == num_sec_){ //Indica que estamos na seção (num_sec_)
				// if(init_urdf_jnts_==-1) init_urdf_jnts_=cnt;
				jointPosVel_.q(ii) = jntmsg->position[cnt];
				jointPosVel_.qdot(ii) = jntmsg->velocity[cnt];
				ii++;
			}
		}
		phi_ = jointPosVel_.q(0); theta_ = jointPosVel_.q(1)*double(num_joints_);
		phi_d_ = jointPosVel_.qdot(0); theta_d_ = double(num_joints_) * jointPosVel_.qdot(1);
		// theta_ é o ângulo TOTAL de flexão
		
		//	Calculando variáveis de configuração (IK_gen)
		
		configPos_ = jnt2screw(Eigen::Vector2d(phi_,theta_)); 
		configVel_ = jacJntCfgA_(Eigen::Vector2d(phi_,theta_)) * Eigen::Vector2d(phi_d_,theta_d_);
		
		kappa_= theta_==0?(0):(theta_/configPos_[2]);
		kappa_d_= (kappa_/2 + 1/2/b_)*jointPosVel_.qdot(1);

		// Calculando cabos (IK_spe)
		cablePosVel_.q.data = ikSpecific(configPos_,a_,alpha_);
		cablePosVel_.qdot.data = jacJntCable_(Eigen::Vector2d(phi_,theta_)) * Eigen::Vector2d(phi_d_,theta_d_);

		// TODO: Cálculo de forças 
		// RCLCPP_INFO_STREAM(get_logger(),"Calling forceJacJntCable_...");
		// cableEffort_.data = Eigen::VectorXd::Constant(num_cables_,preload_);
		// cableEffort_.data += forceJacJntCable_(configPos_) * jointEffort_.data;
			// Com projeção de espaço nulo: adicionar isso
			// + (Eigen::MatrixXd::Identity(num_cables_,num_cables_) - 
			// 	forceJacJntCable_(configPos_) * forceJacCableJnt_(configPos_))
			// 		* auxVector; ///Projeção de espaço nulo
	}

	/**
	 * @brief Processa CableStatesMsg dos cabos.
	 * Passa de cabos para configuração. Passa de configuração para juntas.
	 * 
	 * @param jntmsg ///TODO - renomear para cablesCmdCB - reorganizar para fazer a atuação por torque
	 */
	void ContinuumSection::cablesCB(const continuum_msgs::msg::CableStates::SharedPtr cablemsg){
		RCLCPP_INFO_STREAM(get_logger(),"Cable callback called");
		for (int ii=0; ii<num_cables_; ii++){ //armazena
			cablePosVel_.q(ii) = cablemsg->position[ii];
			cablePosVel_.qdot(ii) = cablemsg->velocity[ii];
			cableEffort_(ii) = cablemsg->effort[ii];
		}
		//	Calculando variáveis de configuração
		configPos_ = fkSpecific(cablePosVel_.q.data(Eigen::seqN(0,4)));
		configVel_ = jacCableCfg_(cablePosVel_.q.data) * cablePosVel_.q.data;
		// Calculando juntas 
		jointPosVel_.q.data = screw2jnt(configPos_);
		jointPosVel_.qdot.data = jacCfgAJnt_(configPos_) * configVel_; 
		jointEffort_.data = forceJacCableJnt_(configPos_) * cableEffort_.data;
	};


	/**
	 * @brief Publica mensagem CableStates com estados dos cabos na seção.
	 * 
	 */
	void ContinuumSection::timerCB(){
		///CableMSG
		continuum_msgs::msg::CableStates cableMsg;
		cableMsg.header.stamp=now();
		cableMsg.section = num_sec_;
		cableMsg.position.resize(num_cables_); cableMsg.velocity.resize(num_cables_);
		
		Eigen::VectorXd::Map(&cableMsg.position[num_sec_*4], num_sec_cables_) = cablePosVel_.q.data;
		Eigen::VectorXd::Map(&cableMsg.velocity[num_sec_*4], num_sec_cables_) = cablePosVel_.qdot.data;
		cableStatePub_->publish(cableMsg);

		//ConfigurationMsg
		continuum_msgs::msg::ConfigStates configMsg;
		configMsg.header.stamp=now();
		configMsg.section = num_sec_;
		Eigen::VectorXd::Map(&configMsg.allen_state[0], 3) = configPos_;
		Eigen::VectorXd::Map(&configMsg.allen_rate[0], 3) = configVel_;
		Eigen::VectorXd::Map(&configMsg.jones_state[0], 3) = Eigen::Vector3d(phi_,kappa_,configPos_[2]);
		Eigen::VectorXd::Map(&configMsg.jones_rate[0], 3) = Eigen::Vector3d(phi_d_,kappa_d_,configVel_[2]);
		configStatePub_->publish(configMsg);

		//Visualização dos cabos
		Eigen::Vector3d p_cabo;
		std::vector<Eigen::Vector3d> cableCentralPt(num_sec_cables_);

		Eigen::AngleAxisd pitchAngle(theta_/2/num_joints_, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yawAngle(phi_ , Eigen::Vector3d::UnitZ());
		Eigen::Quaterniond orientCabos = pitchAngle * yawAngle;

		// Eigen::Isometry3d g = fkIndependent_u(configPos_); // entre vértebras
		// Eigen::Quaterniond orientCabos = Eigen::Quaterniond(g.rotation()).slerp(0.5, Eigen::Quaterniond(Eigen::Matrix3d::Identity()));
		for(int ii=0; ii<num_sec_cables_/4; ii++){ //calcula os comprimentos entre vértebras - eles serão os mesmos para cada seção
			for (int jj=0;jj<4;jj++){
				p_cabo << a_[ii]*cos(double(jj) * M_PI_2 + alpha_[ii] - alpha_[0] - phi_),
						  a_[ii]*sin(double(jj) * M_PI_2 + alpha_[ii] - alpha_[0] - phi_), 0;
				cableCentralPt[ii*4+jj] = p_cabo.topRows(3) +
										orientCabos.toRotationMatrix()*Eigen::Vector3d(0,0,cablePosVel_.q(ii*4+jj)/num_joints_)/2;
				// RCLCPP_INFO_STREAM(get_logger(),"Posição do cabo "<< jj << ": " << std::endl << cableCentralPt[ii*4+jj]);
			}
		}

		// Marcadores dos cabos	
		visualization_msgs::msg::Marker tmp;
		int id=0; int cnt=0;
		for(int ii=0; ii<num_joints_; ii++){
			for(int ss=0; ss<num_sec_cables_/4; ss++){
				for (int jj=0;jj<4;jj++){
					id=ss*4+jj;
					tmp.header.frame_id = "S"+std::to_string(num_sec_)+"D"+std::to_string(ii);
					tmp.frame_locked = true;
					tmp.header.stamp = this->get_clock()->now();
					tmp.ns = "cabled"+std::to_string(ss)+"p"+std::to_string(num_sec_)+"j"+std::to_string(ii);
					tmp.id = id;
					tmp.action = visualization_msgs::msg::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE
					tmp.type = visualization_msgs::msg::Marker::CYLINDER;  // Set our initial shape type to be a sphere
					tmp.pose.position.x = cableCentralPt[id](0);
					tmp.pose.position.y = cableCentralPt[id](1);
					tmp.pose.position.z = cableCentralPt[id](2);
					tmp.pose.orientation.x = orientCabos.x();
					tmp.pose.orientation.y = orientCabos.y();
					tmp.pose.orientation.z = orientCabos.z();
					tmp.pose.orientation.w = orientCabos.w();
					tmp.scale.x = 0.0025;
					tmp.scale.y = 0.0025;
					tmp.scale.z = cablePosVel_.q(id)/num_joints_;
					tmp.color.r = 0.0f;
					tmp.color.g = 0.0f;
					tmp.color.b = 0.0f;
					if(jj==1){
						tmp.color.r = 1.0f;
					}else if(jj==2){
						tmp.color.g = 1.0f;
					}else if(jj==3){
						tmp.color.b = 1.0f;
					}
					tmp.color.a = 1.0; //Alpha deve ser não-zero para ser visível!
					cableMarkerArray_.markers[cnt] = tmp;
					cnt++;
				}
			}
		}
		cableVizPub_->publish(cableMarkerArray_);
	
		//Marcador da seta da curvatura
		visualization_msgs::msg::Marker uvarrow;
		// Eigen::Quaterniond qtmp(Eigen::AngleAxisd(configPos_.norm(),Eigen::Vector3d(configPos_[0],configPos_[1],0)));
		Eigen::Quaterniond qtmp(Eigen::AngleAxisd(M_PI_2 + phi_, Eigen::Vector3d::UnitZ()));
		uvarrow.header.frame_id = "S"+std::to_string(num_sec_)+"B";
		// uvarrow.header.frame_id = "world";
		uvarrow.header.stamp = this->get_clock()->now();
		uvarrow.ns = "allen_sec"+std::to_string(num_sec_);
		uvarrow.id = 0;
		uvarrow.type = visualization_msgs::msg::Marker::ARROW;
		uvarrow.action = visualization_msgs::msg::Marker::ADD;
		uvarrow.pose.position.x =  theta_==0?0:( configPos_[2]/pow(theta_,2)*configPos_[1]);
		uvarrow.pose.position.y =  theta_==0?0:(-configPos_[2]/pow(theta_,2)*configPos_[0]);
		uvarrow.pose.position.z = 0;
		uvarrow.pose.orientation.w = qtmp.w();
		uvarrow.pose.orientation.x = qtmp.x();
		uvarrow.pose.orientation.y = qtmp.y();
		uvarrow.pose.orientation.z = qtmp.z();
		uvarrow.scale.x = configPos_[2]/configPos_.norm();
		uvarrow.scale.y = 0.01;
		uvarrow.scale.z = 0.01;
		uvarrow.color.r = 0.5f;
		uvarrow.color.g = 0.0f;
		uvarrow.color.b = 0.5f;
		uvarrow.color.a = 1.0; //Alpha deve ser não-zero para ser visível!
		configVizPub_->publish(uvarrow);
	}

	/**
	 * @brief Calcula FK independente da seção com base nos parâmetros de configuração recebidos.
	 * 
	 * @param configspace são os parâmetros de configuração da seção - (Phi,Kappa,S) ou (U,V,h)
	 * @param screw escolhe se vou usar transformada do Allen 2020 (se verdadeiro) ou convencional (se falso)
	 * @return Eigen::Isometry3d contendo transformação homogênea do frame distal.
	 */
	Eigen::Isometry3d ContinuumSection::fkIndependent(const Eigen::Vector3d& configspace){
		Eigen::Matrix4d g_; 

		const auto &s = configspace(2);
		double th = sqrt(pow(configspace(0),2)+pow(configspace(1),2));
		double sig = cos(th) - 1;
		double u = configspace(0); double v = configspace(1);
		double ut = u / th; double vt = v / th;
		
		Eigen::Vector3d rho;

		if (fabs(th) >= 1e-6)
		{
			rho << configspace(1) * s / pow(th, 2), -configspace(0) * s / pow(th, 2), 0;

			g_ << sig * pow(vt, 2) + 1, -sig * ut * vt       ,  vt * sin(th), -rho(0) * sig        ,
					-sig * ut * vt       ,  sig * pow(ut, 2) + 1, -ut * sin(th), -rho(1) * sig        ,
					-vt * sin(th)        ,  ut * sin(th)        , cos(th)      ,  rho.norm() * sin(th),
				0					  , 0					 , 0			, 1					   ;
		}
		else
		{ // Aproximação próximo de 0
			rho << -0.5 * s * v, 0.5 * s * u, s;

			g_ << 0.5 * pow(v, 2) + 1, -0.5 * u * v		   ,  v		, rho(0),
					-0.5 * u * v		 ,  0.5 * pow(u, 2) + 1, -u		, rho(1),
					-v					 ,  u				   , cos(th), rho(2),
					0					 ,  0				   , 0		, 1		;
		}
		return Eigen::Isometry3d(g_);
	}


	/**
	 * @brief Implementa a cinemática inversa específica para 4 cabos axialmente simétricos.
	 * Utiliza equações de laço. Considera vários cabos passantes.
	 * 
	 * @param config_params Contém (U,V,h)
	 * @return Eigen::VectorXd contendo valores de comprimento de (num_sec_*4) cabos, ao longo de toda a seção.
	 */
	Eigen::VectorXd ContinuumSection::ikSpecific(const Eigen::Vector3d &config_params,
	 		const Eigen::VectorXd& a,const Eigen::VectorXd& alpha){
		//Cinemática inversa de 4 cabos
		Eigen::Isometry3d g = fkIndependent_u(std::move(config_params)); // entre vértebras
		// RCLCPP_INFO_STREAM(get_logger(), "Unit transform is "<< g.matrix());

		Eigen::Vector4d p_cabo;
		Eigen::VectorXd cable_lengths(num_sec_cables_);
		std::vector<double> betaVec_={0, M_PI_2, M_PI, M_PI + M_PI_2 };
		for (int ss=0;ss<num_sec_cables_/4;ss++){
			for(int ii=0; ii<4; ii++){ //TODO - Vetorializar
				//posição do ponto de fixação do cabo em crds hmg.
				p_cabo << a(ss)*cos(alpha(ss) -alpha(0) + betaVec_[ii]), a(ss)*sin(alpha(ss) - alpha(0) + betaVec_[ii]), 0, 1; 
				// Equação de laço:distância do ponto de fixação em uma
				//  vértebra com a da base, multiplicado pelo num de juntas
				cable_lengths(ii+ss*4) = double(num_joints_) * double(((g*p_cabo - p_cabo).head(3)).norm());
			}
		}
		return cable_lengths;
	}

	/**
	 * @brief Implementa a cinemática direta específica para 4 cabos axialmente simétricos.
	 * 	Utiliza equações como apresentadas em Webster (2010).
	 * 
	 * @param cablelengths Contém comprimentos geométricos de cabos na seção (distais).
	 * @return Eigen::Vector3d contém (Phi,Kappa,S) ou (U,V,h), dependendo de is_screw_.
	 */
	Eigen::Vector3d ContinuumSection::fkSpecific(const Eigen::VectorXd& cablelengths){
		/// Cálcula FK de cabos distais via Eqs. do Webster 2010
		const auto &l1 = cablelengths[0];
		const auto &l2 = cablelengths[1];
		const auto &l3 = cablelengths[2];
		const auto &l4 = cablelengths[3];
		double s;
		phi_ = atan2(l4-l2,l3-l1);
		double g = (l1-3*l2+l3+l4)*sqrt(pow((l4-l2),2)+pow((l3-l1),2));
		// if (fabs(l4-l2)>=1e-6)
		if(fabs(l4-l2)>1e-6){
			kappa_ = g/(a_[0]*(l1+l2+l3+l4)*(l4-l2)); // %Caso l4 !=l2
		}else if(fabs(l4-l2)<=1e-6 && fabs(l3-l1)>=1e-6){
			//l3-l1 = 4*a*n*sin(theta/2n) = 4*a*n*lc*kappa/2
			kappa_ = (l3-l1)/(2*a_[0]*num_joints_*(l1+l3)/2); //Caso l4==l2, então estamos no plano xz - podemos calcular kappa via l3 e l1 só
		}else{
			kappa_ = (l4-l2)/(2*a_[0]*num_joints_*(l2+l4)/2); //checar se isto está ok depois
		}
        s = kappa_==0? 2*b_*num_joints_ : num_joints_ * 2/kappa_ * atan(kappa_*b_);
		
		return jnt2screw(Eigen::Vector2d(phi_,kappa_*s));
	}


	/**
	 * 
	 * @brief Converte de (Phi, Theta) para parametrização do Allen 2020, baseada em (Vetor,ângulo de giro).
	 * 
	 * @param jnt_params é Eigen::Vector3d com parâmetros (Phi, Theta) nessa ordem.
	 * @return Eigen::Vector3d contendo os parâmetros (U,V,s) correspondentes.
	 */
	Eigen::Vector3d ContinuumSection::jnt2screw(const Eigen::Vector2d& jnt_params)
	{
		double u = 0; double v = 0; double s=2*b_*num_joints_;
		Eigen::Vector3d params;
		if(fabs(jnt_params(1))>=1e-5){ // caso não seja singular
			s = jnt_params(1)*b_/tan(jnt_params(1)/2/num_joints_);
			u = -sin(jnt_params(0)) * jnt_params(1);
			v = cos(jnt_params(0)) * jnt_params(1);
		}
		params << u, v, s;
		return params;
	}
	
	/**
	 * @brief Converte de (U,V,h) para (Phi, Kappa, S) tradicionais.
	 * 
	 * @param screw_params é Eigen::Vector3d com parâmetros (U,V,h) do eixo de giro.
	 * @return Eigen::Vector3d com parâmetros (Phi, Kappa, S) correspondentes.
	 */
	Eigen::Vector2d ContinuumSection::screw2jnt(const Eigen::Vector3d& screw_params)
	{
		double phi = (fabs(screw_params(1))>=1e-4||fabs(screw_params(0))>=1e-4)? atan2(screw_params(1),screw_params(0))-M_PI/2 : 0.0;
		double kappa = sqrt(pow(screw_params(0),2)+pow(screw_params(1),2))/screw_params(2); 
		// Eigen::Vector3d config;
		// config << phi, kappa, screw_params(2);
		// return config;
		Eigen::Vector2d config;
		config << phi, kappa * screw_params(2);
		return config;

	}

	Eigen::MatrixXd ContinuumSection::jacJntCable_(const Eigen::Vector2d& jnts)
	{
	//Calcula o Jacobiano de velocidades dos cabos em função de theta e phii
	//No meu caso, eu vou levar em conta os vários momentos aplicados

	Eigen::MatrixXd JacInv(num_sec_cables_,2);
	const auto &ph=jnts(0);
	const auto &th=jnts(1);
	double nj = double(num_joints_);
	double dLdTheta = 0;
	double dLdPhi = 0;
	double al=0;
	double a=0; double ell=0;
	std::vector<double> betaVec_={0, M_PI_2, M_PI, M_PI + M_PI_2 };
	// int sec=0;
	for(int sec=0;sec<(num_sec_cables_/4);sec++){ //define os pontos de fixação dos cabos na seção
		for(int jj=0;jj<4;jj++){
			al=alpha_[sec]+betaVec_[jj];
			ell=cablePosVel_.q.data[jj]/nj;
			a=a_[sec];
			// g = sqrt((2*(1-cos(th))*pow(a_[sec]*cos(alpha_[sec]+betaVec_[jj]-ph)*tan(th/2)-b_,2))/pow(tan(th/2),2));
			// dLdTheta = g * (a_[sec]*cos(alpha_[sec]+betaVec_[jj]-ph)*(1/sin(th)+1/tan(th) + b_*((2-2*cos(th))/pow(sin(th),2)-1)))/
			// 			(2*tan(th/2)*(a_[sec]*cos(alpha_[sec]+betaVec_[jj]-ph)*tan(th/2)-b_));
			// dLdPhi = g * a_[sec] * sin(alpha_[sec]+betaVec_[jj]-ph)*tan(th/2)/(a_[sec]*cos(alpha_[sec]+betaVec_[jj]-ph)*tan(th/2)-b_);
			
			
			dLdPhi = 4*a*sin(ph-al)*(b_*sin(th/nj)-2*a*pow(sin(th/2/nj),2)*cos(ph-al))/(2*ell);
			dLdTheta = 2/nj * (pow(a,2)*sin(th/nj)*pow(cos(ph-al),2) - pow(b_,2)*sin(th/nj) - 2*a*b_*cos(th/nj)*cos(ph-al))/(2*ell);
			JacInv(sec*4+jj,0)=nj*dLdPhi; //Multiplico por nj pois isso é refletido em todos os módulos
			JacInv(sec*4+jj,1)=nj*dLdTheta;
		}
	}
	return JacInv;	
	}

	Eigen::MatrixXd ContinuumSection::jacJntCfgA_(const Eigen::Vector2d &jnts)
	{

	Eigen::MatrixXd Jac(3,2);

	const auto &phi = jnts[0];
	const auto &th  = jnts[1];
	const auto &nj  = num_joints_;

	Jac << -th * cos(phi), -sin(phi),
		   -th * sin(phi),  cos(phi),
		   0, b_/tan(th/2/nj) - b_ * th/(2*nj*(1+pow(th/2/nj,2)/2)*pow(tan(th/2/nj),2));
	return Jac;	
	}


	Eigen::MatrixXd ContinuumSection::jacCfgAJnt_(const Eigen::Vector3d &cfg)
	{

	Eigen::MatrixXd Jac(2,3);
	const auto &u = cfg[0];
	const auto &v = cfg[1];

	Jac << -u/sqrt(pow(u,2)+pow(v,2)), v/sqrt(pow(u,2)+pow(v,2)), 0,
		   -v/(pow(u,2)+pow(v,2)),  u/(pow(u,2)+pow(v,2)), 0;

	return Jac;
	}

	/**
	 * @brief Matriz jacobiana de mapeamento de tensões nos cabos para torques nas juntas. 
	 * 
	 * @param cfg Configuração.
	 * @return Eigen::MatrixXd (6xn_c) com jacobiano de forças
	 */
	Eigen::MatrixXd ContinuumSection::forceJacCableJnt_(const Eigen::Vector3d &cfg)
	{

	Eigen::MatrixXd Jac(6,num_sec_cables_);
	
	Eigen::Vector3d distalPt;
	Eigen::Vector4d proximalPt;
	Eigen::Vector3d cableVector;
	Eigen::Vector3d torqueArm;
	Eigen::Vector3d torqueRes;
	for(int sec=0;sec<num_sec_cables_/4;sec++){ //define os pontos de fixação dos cabos na seção
		for(int jj=0;jj<4;jj++){
			proximalPt=Eigen::Vector4d( //ponto de apoio proximal do cabo
				a_[sec]*cos(alpha_[sec]+jj*M_PI_2),
				a_[sec]*sin(alpha_[sec]+jj*M_PI_2),
				0,
				1);
			distalPt  = (fkIndependent(cfg) * proximalPt)(Eigen::seqN(0,3)); //para distal
			torqueArm = (distalPt - Eigen::Vector3d(0,0,b_));
			
			//vetor do cabo
			cableVector = (proximalPt(Eigen::seqN(0,3))-distalPt).normalized();

			torqueRes = Eigen::Vector3d(torqueArm.cross(cableVector));

			//Colunas do Jacobiano
			Jac(Eigen::seqN(0,3),sec*4+jj) = cableVector;
			Jac(Eigen::seqN(3,3),sec*4+jj) = torqueRes;
			//Jac.col(sec*4+jj) << cableVector.transpose(), torqueRes.transpose();
		}
	}
	RCLCPP_INFO_STREAM(get_logger(),"Jacobian: " << Jac);
	return Jac;
	}
}


int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	//OBRIGATÓRIO: Recebe argumento de número do nodo
	int num_sec_ = 0;
	num_sec_ = std::atoi(argv[1]);

	rclcpp::spin(
		std::make_shared<continuum_manipulator::ContinuumSection>(num_sec_));
	rclcpp::shutdown();
	return 0;
}
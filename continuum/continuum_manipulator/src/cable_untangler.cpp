#ifndef CABLE_UNTANGLER_HPP
#define CABLE_UNTANGLER_HPP
#include <rclcpp/rclcpp.hpp>
#include "continuum_msgs/msg/cable_states.hpp"
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

namespace continuum_manipulator
{
	class CableUnTangler: public rclcpp::Node
	{
	public:
		/// Parâmetros construtivos do robô
		int num_sections_ = 0;					// Número de seções
        // std::vector<int> num_joints_urdf_;      // Número de juntas por seção, de acordo com URDF
		void readParameters();

		KDL::JntArrayVel cablePosVel_;  //Posição e velocidades completas dos cabos
        KDL::JntArray cableEffort_;     //Tensão nos cabos
        std::vector<bool> sectionFlags_; //Flags indicando se lemos de uma seção ou não

		CableUnTangler();
		~CableUnTangler(void);

        //Interface para tangling (receber seções e passar para comprimentos totais)
        rclcpp::Subscription<continuum_msgs::msg::CableStates>::SharedPtr cableStateSub_;   //recebe cabos distais
        rclcpp::Publisher<continuum_msgs::msg::CableStates>::SharedPtr cableStatePub_;      //publica cabos totais
        void cableStatesCB(const continuum_msgs::msg::CableStates::SharedPtr cableSectionStates);

        //Interface para untangling (receber comprimentos totais e passar para seções)
        // rclcpp::Subscription<continuum_msgs::msg::CableStates>::SharedPtr cableCmdSub_; //recebe cabos totais
        // rclcpp::Publisher<continuum_msgs::msg::CableStates>::SharedPtr cableCmdPub_;    //publica cabos distais	
        // void cableCmdCB(const continuum_msgs::msg::CableStates::SharedPtr cableTotalStates);

        //Funções de tangle/untangle
        // std::vector<Eigen::VectorXd> fkUntangle(const std::vector<Eigen::VectorXd> &cablelengths);
        // std::vector<Eigen::VectorXd> ikTangle(const std::vector<Eigen::VectorXd> &);

        //Funções de timer
        rclcpp::TimerBase::SharedPtr timer_;
		double t0_;
        void timerCB();

        //Publisher para visualização dos cabos no RVIZ
        //WIP
	};

    CableUnTangler::CableUnTangler(): Node("cable_untangler"){
        using std::placeholders::_1;
        readParameters();
        cablePosVel_.resize( num_sections_*4 );
        cableEffort_.resize( num_sections_*4 );
        SetToZero(cablePosVel_); SetToZero(cableEffort_);
        sectionFlags_.resize( num_sections_, false);

        //Geração de referência - lê estados de seções, entrega cabos totais
        cableStateSub_=create_subscription<continuum_msgs::msg::CableStates>(
            "/section_cable_states",
            100,
            std::bind(&CableUnTangler::cableStatesCB,this,_1));

        cableStatePub_=create_publisher<continuum_msgs::msg::CableStates>(
            "/cable_states",
            100);

        // //Atuação por cabos - lê cabos totais, passa para cabos distais
        // cableCmdSub_=create_subscription<continuum_msgs::msg::CableStates>(
        //     "/cable_cmd",
        //     100,
        //     std::bind(&CableUnTangler::cableCmdCB,this,_1));

        // cableCmdPub_=create_publisher<continuum_msgs::msg::CableStates>(
        //     "/section_cable_cmd",
        //     100);
        
        t0_=now().seconds();
		using namespace std::chrono_literals;
		timer_=rclcpp::create_timer(this,this->get_clock(),50ms,std::bind(&CableUnTangler::timerCB,this));
    }

    CableUnTangler::~CableUnTangler(){

    }

    void CableUnTangler::readParameters(){
        declare_parameter("num_sec",rclcpp::PARAMETER_INTEGER);
        if(!get_parameter("num_sec",num_sections_))
        {
            RCLCPP_ERROR_STREAM(get_logger(),"No 'num_sec' parameter in node " << get_fully_qualified_name() << ".");
            return;
	    }
        RCLCPP_INFO_STREAM(get_logger(),"Read ''num_sec' as " << num_sections_);

        // num_joints_urdf_.resize(num_sections_);
        // std::vector<long int> tmp;
        // declare_parameter("num_joints_urdf",rclcpp::PARAMETER_INTEGER_ARRAY);
        // if(!get_parameter("num_joints_urdf",tmp))
        // {
        //     RCLCPP_ERROR_STREAM(get_logger(),"No 'num_joints_urdf' parameter in node " << get_fully_qualified_name() << ".");
        //     return;
	    // }
        // for(int ii=0; ii<int(tmp.size());ii++)
        //     num_joints_urdf_[ii] = tmp[ii];
    }

    //Quando recebe os estados dos cabos de cada seção
    //LÓGICA - Vai preenchendo com os estados de cada seção. Quando tiver recebido todos, publica.
    //Isso ocorre no callback do timer.
    void CableUnTangler::cableStatesCB(const continuum_msgs::msg::CableStates::SharedPtr cablemsg)
    {

        int sec = int(cablemsg->section); //seção de qual recebemos mensagem

        //Isso é uma versão simplificada do algoritmo de entrelaçamento
        // A solução das parcelas dos comprimentos totais já ocorreu nos nodos
        //  de cada seção - portanto, não precisamos solucionar de novo
        // É só somar as posições/velocidades para obter as posições/velocidades totais
        
        for (int ii=4*sec; ii<num_sections_*4; ii++){
            cablePosVel_.q(ii) += cablemsg->position[ii];
            // RCLCPP_INFO_STREAM(get_logger(),"S | ii | q | qdot: " << sec  << " " << ii << " " << cablemsg->position[ii] << " " << cablemsg->velocity[ii]);
            cablePosVel_.qdot(ii) += cablemsg->velocity[ii];
        }
    
        sectionFlags_[sec] = true; //lemos dessa seção
    }

    // //Quando recebe um comando - comprimento de cabos totais
    // void CableUnTangler::cableCmdCB(const continuum_msgs::msg::CableStates::SharedPtr cablemsg)
    // {
    //     continuum_msgs::msg::CableStates msg;
    //     //callback de estados
    //     int cntjnts=0;

    //     for (int ii=0; ii<num_sections_; ii++){
    //         msg.name.resize(1);
    //         msg.name[0] = "continuum_section_"+std::to_string(ii);
    //         msg.position.resize(num_joints_urdf_[ii]);
    //         msg.velocity.resize(num_joints_urdf_[ii]);
    //         for (int jj=0; jj<num_joints_urdf_[ii];jj++){
    //             msg.position[jj] = cablePosVel_.q.data(cntjnts);
    //             msg.velocity[jj] = cablePosVel_.qdot.data(cntjnts);
    //             cntjnts++;
    //         }
    //         jointStatePubArray_[ii]->publish(msg);
    //     }
    // }

    //Loop
    void CableUnTangler::timerCB(void)
	{
        // RCLCPP_INFO_STREAM(get_logger(),"Timer callbck ");
        RCLCPP_INFO_STREAM(get_logger()," q: \n" << cablePosVel_.q.data.transpose()  << "\n qdot: \n" << cablePosVel_.qdot.data.transpose() << std::endl);
        if (std::all_of(sectionFlags_.begin(), sectionFlags_.end(), [](bool x) { return x; } )){
            //Parte que processa o caminho de geração de referência
            continuum_msgs::msg::CableStates cableMsg;
            cableMsg.header.frame_id = "world";
            cableMsg.header.stamp=now();

            cableMsg.section = 0;
            cableMsg.position.resize(cablePosVel_.q.data.size());
            cableMsg.velocity.resize(cablePosVel_.qdot.data.size());
            cableMsg.effort.resize(cableEffort_.data.size());
            Eigen::VectorXd::Map(&cableMsg.position[0], cablePosVel_.q.data.size()) = cablePosVel_.q.data;
            Eigen::VectorXd::Map(&cableMsg.velocity[0], cablePosVel_.qdot.data.size()) = cablePosVel_.qdot.data;
            Eigen::VectorXd::Map(&cableMsg.effort[0], cableEffort_.data.size()) = cableEffort_.data;

            cableStatePub_->publish(cableMsg);
            SetToZero(cablePosVel_);
            std::fill(sectionFlags_.begin(), sectionFlags_.end(), false);
        }
		else{
            return;
        }
	}
}

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<continuum_manipulator::CableUnTangler>());
    rclcpp::shutdown();
    return 0;
}
#endif
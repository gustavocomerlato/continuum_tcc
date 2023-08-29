/******************************************************************************
	           ROS 2 Cartesian Trajectory Generation Example
          Copyright (C) 2018, 2021 Walter Fetter Lages <w.fetter@ieee.org>

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.
        
*******************************************************************************/

//** Modified by Gustavo Comerlato Rodrigues @08/05/2023

#include <rclcpp/rclcpp.hpp>

#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_cyclic_closed.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/utilities/error.h>
#include <tf2_kdl/tf2_kdl.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>

using namespace KDL;

class PoseTrajectory: public rclcpp::Node
{
	public:
	PoseTrajectory(const char *name="trajectory_publisher");
	
	private:
	Trajectory_Composite trajectory_;
	double t0_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twistPublisher_;
	void timerCB(void) const;
};

PoseTrajectory::PoseTrajectory(const char *name): Node(name)
{
	try
	{
		//TRAJETÓRIA TESTE 1: Caminho compósito por Via Points
		// auto path=new Path_RoundedComposite(0.02,0.002,new RotationalInterpolation_SingleAxis());
		// path->Add(Frame(Rotation::RPY(0,0,0),Vector(0.0,0.0,0.9)));
		// path->Add(Frame(Rotation::RotY(-M_PI_2),Vector(0.1,-0.1,0.7)));
		// path->Add(Frame(Rotation::RotZ(-M_PI_2),Vector(0.2,-0.1,0.6)));
		// path->Add(Frame(Rotation::Quaternion(0,0,0.375,0.926),Vector(0.437,0.424,0.4)));
		// path->Add(Frame(Rotation::RotZ(M_PI_2*0),Vector(0.238,0.505,0.4)));
		// path->Add(Frame(Rotation::RPY(0,20,0),Vector(0.61,0,0.1)));
		// path->Finish();

		// Velocidade e aceleração
		// auto velocityProfile=new VelocityProfile_Trap(0.1,0.02);  
		// velocityProfile->SetProfile(0,circlePath->PathLength());
		// // Junta os dois para formar a trajetória
		// auto trajectorySegment=new Trajectory_Segment(circlePath,velocityProfile);
		// // auto trajectoryStationary=new Trajectory_Stationary(1.0,Frame(Rotation::RPY(0,20,0),Vector(0.61,0,0.1)));
		// trajectory_.Add(trajectorySegment);
		// // trajectory_.Add(trajectoryStationary);
		
		//TRAJETÓRIA TESTE 2: Frame
		// auto circlePath = new Path_RoundedComposite(0.02,0.002,)
		// double cz=0.3;	double csx=0.3;	double totalrotationangle=50*M_PI;
		
		// auto circlePath = new Path_Circle(
		// 					Frame(Rotation::RPY(0,30*M_PI/180,0),Vector(csx,0,cz)),
		// 					Vector(0,0,cz),
		// 					Vector(0,1,cz),
		// 					Rotation::RPY(0,30*M_PI/180,0),
		// 					totalrotationangle,
		// 					new RotationalInterpolation_SingleAxis(),1);

		// // Velocidade e aceleração
		// auto velocityProfile=new VelocityProfile_Trap(0.1,0.02);  
		// velocityProfile->SetProfile(0,circlePath->PathLength());

		// // Junta os dois para formar a trajetória
		// auto trajectorySegment=new Trajectory_Segment(circlePath,velocityProfile);
		// trajectory_.Add(trajectorySegment);

		//TRAJETÓRIA TESTE 3: Hélice
		// auto circlePath = new Path_RoundedComposite(0.02,0.002,)
		double rad=0.15; double pitch=0.08; double h0=0.01; double hf=0.3;
		double nHelix=(hf-h0)/pitch; double t;
		int nPts=300; KDL::Vector viapt;

		auto path=new Path_RoundedComposite(0.02,0.002,new RotationalInterpolation_SingleAxis());
		for(int ii=0;ii<=nPts;ii++){
			t = nHelix/double(nPts)*double(ii);
			viapt.data[0] = (rad)*cos(2*M_PI*t);
			viapt.data[1] = (rad)*sin(2*M_PI*t);
			viapt.data[2] = pitch*t+h0;
			path->Add(Frame(Rotation::RPY(0,0,0),viapt));
		}
		path->Finish();

		// Velocidade e aceleração
		auto velocityProfile=new VelocityProfile_Trap(0.1,0.02);  
		velocityProfile->SetProfile(0,path->PathLength());

		// Junta os dois para formar a trajetória
		auto trajectoryStationary=new Trajectory_Stationary(10.0,Frame(Rotation::RPY(0,0,0),Vector(rad,0,h0)));
		trajectory_.Add(trajectoryStationary);
		auto trajectorySegment=new Trajectory_Segment(path,velocityProfile);
		trajectory_.Add(trajectorySegment);


		// //TRAJETÓRIA TESTE 3: Frame pivotando em torno de um eixo central
		// double cz=0.3;	double rad=0.3;	double totalRotationangle=4*M_PI;
		// int nArcSegments = 60; double arcLength=totalRotationangle/double(nArcSegments);
		// for(int ii=0;ii<=nArcSegments;ii++){
			
		// 	Eigen::Quaterniond nextFrame = Eigen::Quaterniond(startFrame)
		// 	Eigen::Quaterniond(g.rotation()).slerp(0.5, Eigen::AngleAxisd());
		// 	angle = Eigen::Quaterniond(g.rotation()).slerp(0.5, Eigen::Quaterniond(Eigen::Matrix3d::Identity()))
		// 	KDL::Vector startPt(rad*cos(angle),rad*sin(angle),cz);
		// 	auto circlePath = new Path_Circle(
		// 			Frame(Rotation::RPY(0,30*M_PI/180,0),Vector(csx,0,cz)),
		// 			Vector(0,0,cz),
		// 			Vector(0,1,cz),
		// 			Rotation::RPY(0,30*M_PI/180,0),
		// 			totalrotationangle,
		// 			new RotationalInterpolation_SingleAxis(),1);

		// 	auto trajectorySegment = new Trajectory_Segment(circlePath,velocityProfile);
		// 	trajectory_.Add(trajectorySegment);
		// }
	}
	catch(Error &error)
	{
		RCLCPP_ERROR_STREAM(get_logger(),"Error: " << error.Description() << std::endl);
		RCLCPP_ERROR_STREAM(get_logger(),"Type: " << error.GetType() << std::endl);
	}

	posePublisher_=create_publisher<geometry_msgs::msg::PoseStamped>("traj_pose",100);
	twistPublisher_=create_publisher<geometry_msgs::msg::TwistStamped>("traj_twist",100);

	t0_=now().seconds();
	using namespace std::chrono_literals;
	timer_=rclcpp::create_timer(this,this->get_clock(),50ms,std::bind(&PoseTrajectory::timerCB,this));
}

void PoseTrajectory::timerCB(void) const
{
	double t=fmin(now().seconds()-t0_,trajectory_.Duration());
	tf2::Stamped<KDL::Frame> pose(trajectory_.Pos(t),tf2::get_now(),"world");
	tf2::Stamped<KDL::Twist> twist(trajectory_.Vel(t),tf2::get_now(),"world");
	auto poseMsg=tf2::toMsg(pose);
	auto twistMsg=tf2::toMsg(twist);
	posePublisher_->publish(poseMsg);
	twistPublisher_->publish(twistMsg);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PoseTrajectory>());
	rclcpp::shutdown();
	return 0;
}

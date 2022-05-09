#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream> 
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <functional>
#include <ignition/math/Vector3.hh>

#include "Eigen/Dense"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
        
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Model* kubot22_Model = new Model;
VectorXd calG(12);
VectorXd calC(12);


RigidBodyDynamics::Math::VectorNd RobotState(12);
RigidBodyDynamics::Math::VectorNd RobotStatedot(12);
RigidBodyDynamics::Math::VectorNd RobotStatedot2(12);


VectorXd Tau = VectorXd::Zero(12);
VectorXd Nowq = VectorXd::Zero(12);
VectorXd NowDq = VectorXd::Zero(12);
VectorXd NowD2q = VectorXd::Zero(12);


float LJ[6] = {0};
float RJ[6] = {0};
 
float N_LJ[6] = {0};
float N_RJ[6] = {0};
 
int CNT = 0;
int T = 1000;

#define PI      3.141592
#define D2R     3.141592/180
#define R2D     180/PI


void GetJoint(const std_msgs::Float64MultiArray &msg);

ros::Subscriber GET_JOINT;


namespace gazebo{
    class PIDJoints : public ModelPlugin
    {
        double dt;

        double LP_ANGLE           = 0;
        double LPm_ANGLE   	  = 0;
        double LPd_ANGLE 	  = 0;
        double LK_ANGLE           = 0;
        double LA_ANGLE           = 0;
        double LF_ANGLE       	  = 0;
        
        double RP_ANGLE           = 0;
        double RPm_ANGLE   	  = 0;
        double RPd_ANGLE 	  = 0;
        double RK_ANGLE           = 0;
        double RA_ANGLE           = 0;
        double RF_ANGLE       	  = 0;
        

	double LP_ANGLE_er        = 0;
        double LPm_ANGLE_er   	  = 0;
        double LPd_ANGLE_er	  = 0;
        double LK_ANGLE_er        = 0;
        double LA_ANGLE_er        = 0;
        double LF_ANGLE_er        = 0;
        
        double RP_ANGLE_er        = 0;
        double RPm_ANGLE_er   	  = 0;
        double RPd_ANGLE_er	  = 0;
        double RK_ANGLE_er        = 0;
        double RA_ANGLE_er        = 0;
        double RF_ANGLE_er        = 0;

        physics::LinkPtr base_link     ;
        physics::LinkPtr L_P_link      ;
        physics::LinkPtr L_Pm_link     ;
        physics::LinkPtr L_Pd_link     ;
        physics::LinkPtr L_K_link      ;
        physics::LinkPtr L_A_link      ;
        physics::LinkPtr L_F_link      ;
        
        physics::LinkPtr R_P_link      ;
        physics::LinkPtr R_Pm_link     ;
        physics::LinkPtr R_Pd_link     ;
        physics::LinkPtr R_K_link      ;
        physics::LinkPtr R_A_link      ;
        physics::LinkPtr R_F_link      ;
        
        
        physics::JointPtr LP           ;
        physics::JointPtr LPm          ;
        physics::JointPtr LPd          ;
        physics::JointPtr LK           ;
        physics::JointPtr LA           ;
        physics::JointPtr LF           ;
       
       
        physics::JointPtr RP           ;
        physics::JointPtr RPm          ;
        physics::JointPtr RPd          ;
        physics::JointPtr RK           ;
        physics::JointPtr RA           ;
        physics::JointPtr RF           ;
        

        
        common::PID LPpid              ;
        common::PID LPmpid             ;
        common::PID LPdpid             ;
        common::PID LKpid              ;
        common::PID LApid              ;
        common::PID LFpid              ;
        
        common::PID RPpid              ;
        common::PID RPmpid             ;
        common::PID RPdpid             ;
        common::PID RKpid              ;
        common::PID RApid              ;
        common::PID RFpid              ;
        
        
        physics::ModelPtr model;

        common::Time last_update_time;
        event::ConnectionPtr update_connection_;

        ros::NodeHandle n;
        ros::Publisher P_Times;
        
     
        public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdatePID();
        
};

    GZ_REGISTER_MODEL_PLUGIN(PIDJoints);
    
}

void gazebo::PIDJoints::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) 
{
    
    //cout << b.segment(3, 2) << endl;
    //cout << "B" << endl;
    this->model = _model;
    
    model = _model;
    
    this->base_link      = this->model->GetLink("base_link");
    
    this->L_P_link       = this->model->GetLink("L_P_link");
    this->L_Pm_link   	 = this->model->GetLink("L_Pm_link");
    this->L_Pd_link 	 = this->model->GetLink("L_Pd_link");
    this->L_K_link       = this->model->GetLink("L_K_link");
    this->L_A_link       = this->model->GetLink("L_A_link");
    this->L_F_link       = this->model->GetLink("L_F_link");
    
    this->R_P_link       = this->model->GetLink("R_P_link");
    this->R_Pm_link  	 = this->model->GetLink("R_Pm_link");
    this->R_Pd_link 	 = this->model->GetLink("R_Pd_link");
    this->R_K_link       = this->model->GetLink("R_K_link");
    this->R_A_link     	 = this->model->GetLink("R_A_link");
    this->R_F_link       = this->model->GetLink("R_F_link");
    
    
    this->LP             = this->model->GetJoint("LP");
    this->LPm            = this->model->GetJoint("LPm");
    this->LPd            = this->model->GetJoint("LPd");
    this->LK             = this->model->GetJoint("LK");
    this->LA             = this->model->GetJoint("LA");
    this->LF             = this->model->GetJoint("LF");
         
    this->RP 	         = this->model->GetJoint("RP");
    this->RPm   	 = this->model->GetJoint("RPm");
    this->RPd     	 = this->model->GetJoint("RPd");
    this->RK             = this->model->GetJoint("RK");
    this->RA             = this->model->GetJoint("RA");
    this->RF             = this->model->GetJoint("RF");
    

   
    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PIDJoints::UpdatePID, this));

    this->LPpid.Init( 1.2, 0, 0.1, 200, -200, 1000, -1000);
    this->LPmpid.Init(2.4, 0, 0.2, 200, -200, 1000, -1000);
    this->LPdpid.Init(2.4, 0, 0.2, 200, -200, 1000, -1000);
    this->LKpid.Init( 4.8, 0, 0.4, 200, -200, 1000, -1000);
    this->LApid.Init( 3.6, 0, 0.3, 200, -200, 1000, -1000);
    this->LFpid.Init( 2.4, 0, 0.2, 200, -200, 1000, -1000);
    
    this->RPpid.Init( 1.2, 0, 0.1, 200, -200, 1000, -1000);
    this->RPmpid.Init(2.4, 0, 0.2, 200, -200, 1000, -1000);
    this->RPdpid.Init(2.4, 0, 0.2, 200, -200, 1000, -1000);
    this->RKpid.Init( 4.8, 0, 0.4, 200, -200, 1000, -1000);
    this->RApid.Init( 3.6, 0, 0.3, 200, -200, 1000, -1000);
    this->RFpid.Init( 2.4, 0, 0.2, 200, -200, 1000, -1000);
    
  
    
    int version_test;
    
    version_test = rbdl_get_api_version();
    
     printf("RBDL API version = %d\n", version_test);
     
    Addons::URDFReadFromFile("/home/yesun/.gazebo/models/kubot22/urdf/kubot22.urdf", kubot22_Model, false, false);
     
    kubot22_Model -> gravity = Eigen::Vector3d(0., 0., -9.81);
     
    RobotState = VectorNd::Zero(kubot22_Model->q_size);
    RobotStatedot = VectorNd::Zero(kubot22_Model->q_size);
    RobotStatedot2 = VectorNd::Zero(kubot22_Model->q_size);

    
    GET_JOINT = n.subscribe("desQ_array", 100, GetJoint);
    
    
    ros::Rate loop_rate(1000);
}

void gazebo::PIDJoints::UpdatePID()//여러번 실행
{
    //cout << "A" << endl;
    common::Time current_time = this->model->GetWorld()->GetSimTime();
    dt = current_time.Double() - this->last_update_time.Double();
   
    this->last_update_time = current_time;

    LP_ANGLE           = this-> LP          ->GetAngle(2).Radian();
    LPm_ANGLE          = this-> LPm         ->GetAngle(1).Radian();
    LPd_ANGLE          = this-> LPd 	    ->GetAngle(1).Radian();
    LK_ANGLE           = this-> LK          ->GetAngle(1).Radian();
    LA_ANGLE           = this-> LA          ->GetAngle(1).Radian();
    LF_ANGLE           = this-> LF          ->GetAngle(1).Radian();
    
    RP_ANGLE           = this-> RP          ->GetAngle(2).Radian();
    RPm_ANGLE          = this-> RPm         ->GetAngle(1).Radian();
    RPd_ANGLE          = this-> RPd 	    ->GetAngle(1).Radian();
    RK_ANGLE           = this-> RK          ->GetAngle(1).Radian();
    RA_ANGLE           = this-> RA          ->GetAngle(1).Radian();
    RF_ANGLE           = this-> RF          ->GetAngle(1).Radian();
    
    //one motion cyclei
    if(LJ[0] != N_LJ[0] || RJ[0] != N_RJ[0]){
        if(CNT == 0){
            printf("start");
            CNT ++; 
        }else if(CNT < T){
        LP_ANGLE_er  = LP_ANGLE -  (((LJ[0]-N_LJ[0])*(1-cos(PI*CNT/T)))/2+N_LJ[0])*D2R;
        LPm_ANGLE_er = LPm_ANGLE - (((LJ[1]-N_LJ[1])*(1-cos(PI*CNT/T)))/2+N_LJ[1])*D2R;
        LPd_ANGLE_er = LPd_ANGLE - (((LJ[2]-N_LJ[2])*(1-cos(PI*CNT/T)))/2+N_LJ[2])*D2R;
        LK_ANGLE_er  = LK_ANGLE -  (((LJ[3]-N_LJ[3])*(1-cos(PI*CNT/T)))/2+N_LJ[3])*D2R;
        LA_ANGLE_er  = LA_ANGLE -  (((LJ[4]-N_LJ[4])*(1-cos(PI*CNT/T)))/2+N_LJ[4])*D2R;
        LF_ANGLE_er  = LF_ANGLE -  (((LJ[5]-N_LJ[5])*(1-cos(PI*CNT/T)))/2+N_LJ[5])*D2R;

        RP_ANGLE_er  = RP_ANGLE -  (((RJ[0]-N_RJ[0])*(1-cos(PI*CNT/T)))/2+N_RJ[0])*D2R;
        RPm_ANGLE_er = RPm_ANGLE - (((RJ[1]-N_RJ[1])*(1-cos(PI*CNT/T)))/2+N_RJ[1])*D2R;
        RPd_ANGLE_er = RPd_ANGLE - (((RJ[2]-N_RJ[2])*(1-cos(PI*CNT/T)))/2+N_RJ[2])*D2R;
        RK_ANGLE_er  = RK_ANGLE -  (((RJ[3]-N_RJ[3])*(1-cos(PI*CNT/T)))/2+N_RJ[3])*D2R;
        RA_ANGLE_er  = RA_ANGLE -  (((RJ[4]-N_RJ[4])*(1-cos(PI*CNT/T)))/2+N_RJ[4])*D2R;
        RF_ANGLE_er  = RF_ANGLE -  (((RJ[5]-N_RJ[5])*(1-cos(PI*CNT/T)))/2+N_RJ[5])*D2R;
        CNT ++; 
        }else{
            CNT = 0;
            N_LJ[0] = LJ[0];  N_RJ[0] = RJ[0];
            N_LJ[1] = LJ[1];  N_RJ[1] = RJ[1];
            N_LJ[2] = LJ[2];  N_RJ[2] = RJ[2];
            N_LJ[3] = LJ[3];  N_RJ[3] = RJ[3];
            N_LJ[4] = LJ[4];  N_RJ[4] = RJ[4];
            N_LJ[5] = LJ[5];  N_RJ[5] = RJ[5];
            
            LP_ANGLE_er  = LP_ANGLE -  N_LJ[0]*D2R;
            LPm_ANGLE_er = LPm_ANGLE - N_LJ[1]*D2R;
            LPd_ANGLE_er = LPd_ANGLE - N_LJ[2]*D2R;
            LK_ANGLE_er  = LK_ANGLE -  N_LJ[3]*D2R;
            LA_ANGLE_er  = LA_ANGLE -  N_LJ[4]*D2R;
            LF_ANGLE_er  = LF_ANGLE -  N_LJ[5]*D2R;

            RP_ANGLE_er  = RP_ANGLE -  N_RJ[0]*D2R;
            RPm_ANGLE_er = RPm_ANGLE - N_RJ[1]*D2R;
            RPd_ANGLE_er = RPd_ANGLE - N_RJ[2]*D2R;
            RK_ANGLE_er  = RK_ANGLE -  N_RJ[3]*D2R;
            RA_ANGLE_er  = RA_ANGLE -  N_RJ[4]*D2R;
            RF_ANGLE_er  = RF_ANGLE -  N_RJ[5]*D2R;
        }
       
           
    }else{
    LP_ANGLE_er  = LP_ANGLE -  N_LJ[0]*D2R;
    LPm_ANGLE_er = LPm_ANGLE - N_LJ[1]*D2R;
    LPd_ANGLE_er = LPd_ANGLE - N_LJ[2]*D2R;
    LK_ANGLE_er  = LK_ANGLE -  N_LJ[3]*D2R;
    LA_ANGLE_er  = LA_ANGLE -  N_LJ[4]*D2R;
    LF_ANGLE_er  = LF_ANGLE -  N_LJ[5]*D2R;

    RP_ANGLE_er  = RP_ANGLE -  N_RJ[0]*D2R;
    RPm_ANGLE_er = RPm_ANGLE - N_RJ[1]*D2R;
    RPd_ANGLE_er = RPd_ANGLE - N_RJ[2]*D2R;
    RK_ANGLE_er  = RK_ANGLE -  N_RJ[3]*D2R;
    RA_ANGLE_er  = RA_ANGLE -  N_RJ[4]*D2R;
    RF_ANGLE_er  = RF_ANGLE -  N_RJ[5]*D2R;
    }
    this -> LPpid.Update(LP_ANGLE_er, dt);
    this -> LPmpid.Update(LPm_ANGLE_er, dt);
    this -> LPdpid.Update(LPd_ANGLE_er, dt);
    this -> LKpid.Update(LK_ANGLE_er, dt);
    this -> LApid.Update(LA_ANGLE_er, dt);
    this -> LFpid.Update(LF_ANGLE_er, dt);
  
    this -> RPpid.Update(RP_ANGLE_er, dt);
    this -> RPmpid.Update(RPm_ANGLE_er, dt);
    this -> RPdpid.Update(RPd_ANGLE_er, dt);
    this -> RKpid.Update(RK_ANGLE_er, dt);
    this -> RApid.Update(RA_ANGLE_er, dt);
    this -> RFpid.Update(RF_ANGLE_er, dt);


    
    for (int LnJoint = 0; LnJoint < 12; LnJoint++) {
        RobotState(LnJoint)     = NowDq(LnJoint);
        RobotStatedot(LnJoint)  = NowDq(LnJoint);
        RobotStatedot2(LnJoint) = NowD2q(LnJoint);
    }
    
//    VectorNd hatNonLinearEffects = VectorNd::Zero(kubot22_Model->dof_count);
//    NonlinearEffects(*kubot22_Model, RobotState, RobotStatedot, hatNonLinearEffects); //C(q,q')+G(q)
//
//    VectorXd L_G = VectorXd::Zero(kubot22_Model->dof_count);
//    NonlinearEffects(*kubot22_Model, RobotState, VectorNd::Zero(kubot22_Model->dof_count), L_G); //G(q)
//
//    calG << L_G(0), L_G(1), L_G(2), L_G(3), L_G(4), L_G(5), L_G(6), L_G(7), L_G(8), L_G(9), L_G(10), L_G(11);
//
//    calC << (hatNonLinearEffects(0) - L_G(0)), (hatNonLinearEffects(1) - L_G(1)), (hatNonLinearEffects(2) - L_G(2)),
//              (hatNonLinearEffects(3) - L_G(3)), (hatNonLinearEffects(4) - L_G(4)), (hatNonLinearEffects(5) - L_G(5)),
//              (hatNonLinearEffects(6) - L_G(6)), (hatNonLinearEffects(7) - L_G(7)), (hatNonLinearEffects(8) - L_G(8)),
//              (hatNonLinearEffects(9) - L_G(9)), (hatNonLinearEffects(10) - L_G(10)), (hatNonLinearEffects(11) - L_G(11));
//    
//    Tau << 0.999 * calG+ calC ;
//    Tau << 0.999 * calG ;
    
      
    
    LP ->SetForce(2, LPpid.GetCmd()); //setForce(axis,Force value)
    LPm->SetForce(1, LPmpid.GetCmd());
    LPd->SetForce(1, LPdpid.GetCmd());
    LK ->SetForce(1, LKpid.GetCmd());
    LA ->SetForce(1, LApid.GetCmd());
    LF ->SetForce(1, LFpid.GetCmd());

    RP ->SetForce(2, RPpid.GetCmd()); //setForce(axis,Force value)
    RPm->SetForce(1, RPmpid.GetCmd());
    RPd->SetForce(1, RPdpid.GetCmd());
    RK ->SetForce(1, RKpid.GetCmd());
    RA ->SetForce(1, RApid.GetCmd());
    RF ->SetForce(1, RFpid.GetCmd());

    
    //this->LP ->SetForce(2, -Tau(0)); //setForce(axis,Force value)
    //this->LPm->SetForce(1, -Tau(1));
    //this->LPd->SetForce(1, -Tau(2));
    //this->LK ->SetForce(1, -Tau(3));
    //this->LA ->SetForce(1, -Tau(4));
    //this->LF ->SetForce(1, -Tau(5));

    //this->RP ->SetForce(2, Tau(6) ); //setForce(axis,Force value)
    //this->RPm->SetForce(1, Tau(7) );
    //this->RPd->SetForce(1, Tau(8) );
    //this->RK ->SetForce(1, Tau(9) );
    //this->RA ->SetForce(1, Tau(10));
    //this->RF ->SetForce(1, Tau(11));
    
    
    if (CNT%1000 == 0){
        //printf("LP_ANGLE_er = %f\n" , LP_ANGLE*R2D );
        //printf("LPm_ANGLE_er = %f\n" , LPm_ANGLE*R2D);
        //printf("LPd_ANGLE_er = %f\n" , LPd_ANGLE*R2D);
        //printf("LK_ANGLE_er = %f\n" , LK_ANGLE*R2D );
        //printf("LA_ANGLE_er = %f\n" , LA_ANGLE*R2D );
        //printf("LF_ANGLE_er = %f\n" , LF_ANGLE*R2D );
        //
        //printf("RP_ANGLE_er = %f\n" , RP_ANGLE*R2D);
        //printf("RPm_ANGLE_er = %f\n" , RPm_ANGLE*R2D);
        //printf("RPd_ANGLE_er = %f\n" , RPd_ANGLE*R2D);
        //printf("RK_ANGLE_er = %f\n" , RK_ANGLE*R2D);
        //printf("RA_ANGLE_er = %f\n" , RA_ANGLE*R2D);
        //printf("RF_ANGLE_er = %f\n" , RF_ANGLE*R2D);
    }
    
}


void GetJoint(const std_msgs::Float64MultiArray &msg)
{
    LJ[0] = msg.data[0];
    LJ[1] = msg.data[1];
    LJ[2] = msg.data[2];
    LJ[3] = msg.data[3];
    LJ[4] = msg.data[4];
    LJ[5] = msg.data[5];

    RJ[0] = msg.data[6];
    RJ[1] = msg.data[7];
    RJ[2] = msg.data[8];
    RJ[3] = msg.data[9];
    RJ[4] = msg.data[10];
    RJ[5] = msg.data[11];
}

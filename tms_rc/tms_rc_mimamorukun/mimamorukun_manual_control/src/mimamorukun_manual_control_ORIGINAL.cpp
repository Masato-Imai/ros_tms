//------------------------------------------------------------------------------
// FileName : mimamorukun_manual_control.cpp
// Date     : 2014.11.07
// author   : Akio Shigekane
//------------------------------------------------------------------------------
//2014. 9.23        adjust scale of speed
//2014.11.01        adjust wheel spin PID constants
//2014.11.07        included to ROT_TMS project

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <string>

#include <sensor_msgs/Joy.h>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_rc/rc_robot_control.h>

#define ROS_RATE   10

using namespace std;

ClientSocket  client_socket (""/*"192.168.11.99"*/, 54300 );
const int     ENC_MAX  = 3932159;
const int     SPEED_MAX = 32767;
const float   DIST_PER_PULSE = 0.552486;  //mm par pulse
const int     WHEEL_DIST = 533;

long int    ENC_L = 0;
long int    ENC_R = 0;
int         POS_X = 0;
int         POS_Y = 0;
// double   POS_SIGMA = 0;
// double   POS_ANG = 0;

/*double joy_cmd_spd = 0.0;
double joy_cmd_turn = 0.0;*/

ros::ServiceClient db_client;

class MachinePose_s{
private:
public:
    MachinePose_s() {};
    ~MachinePose_s() {};
    void updateOdom();
    void updateVicon();
    void updateCompFilter();
    bool goPose(/*const geometry_msgs::Pose2D::ConstPtr& cmd_pose*/);
    geometry_msgs::Pose2D   tgtPose;
    geometry_msgs::Twist    tgtTwist;
    geometry_msgs::Pose2D   pos_odom;
    geometry_msgs::Pose2D   pos_vicon;
    geometry_msgs::Pose2D   pos_fusioned;
/*    geometry_msgs::Twist    vel_odom;
    geometry_msgs::Twist    vel_vicon;
    geometry_msgs::Twist    vel_fusioned;*/
    geometry_msgs::Pose2D vel_odom;
    geometry_msgs::Pose2D vel_vicon;
    geometry_msgs::Pose2D vel_fusioned;
}mchn_pose;


int Dist2Pulse(int dist){   return ((float)dist)/DIST_PER_PULSE;}
int Pulse2Dist(int pulse){  return ((float)pulse)*DIST_PER_PULSE;}
double Rad2Deg(double rad){ return rad*(180.0)/M_PI;}
// double Deg2Rad(double deg){ return deg*M_PI/180.0;}
double Deg2Rad(double deg){ return deg*3/180.0;}
double MM2M(double mm){     return mm*0.001;}
double M2MM(double M){      return M*1000;}
double sqr(double val){            return pow(val,2);}
double Limit(double val,double max,double min){
    if(val > max)       return max;
    else if(min > val)  return min;
    else                return val;
}
double nomalizeAng(double rad){
    while(rad > M_PI){  //角度を-180°~180°(-π~π)の範囲に合わせる
        rad = rad - (2*M_PI);
    }
    while(rad < -M_PI){
        rad = rad + (2*M_PI);
    }
    return rad;
}

void MachinePose_s::updateVicon(){
    // printf("line:%s\n",__LINE__);
    tms_msg_db::TmsdbGetData srv;
    srv.request.tmsdb.id = 2007;
    if(! db_client.call(srv)){
        ROS_ERROR("Failed to get vicon data from DB via tms_db_reader");
    }else if(srv.response.tmsdb.empty()){
        ROS_ERROR("DB response empty");
    }else{
        this->pos_vicon.x = srv.response.tmsdb[0].x;
        this->pos_vicon.y = srv.response.tmsdb[0].y;
        this->pos_vicon.theta = Deg2Rad(srv.response.tmsdb[0].ry);
    }
    return ;
}

void MachinePose_s::updateOdom(){
    //update Encoder value
    long int tmpENC_L = 0;
    long int tmpENC_R = 0;
    string reply;
    client_socket << "@GP1@GP2";    /*use 250ms for send and get reply*/
    client_socket >> reply;          
    cout << "Response:" << reply << "\n";
    sscanf(reply.c_str(),"@GP1,%ld@GP2,%ld",&tmpENC_L,&tmpENC_R);
    if(tmpENC_L > ENC_MAX/2)ENC_L = tmpENC_L-(ENC_MAX+1);
    else                    ENC_L = tmpENC_L;
    if(tmpENC_R > ENC_MAX/2)ENC_R = tmpENC_R-(ENC_MAX+1);
    else                    ENC_R = tmpENC_R;

    static long int ENC_R_old = 0;static long int ENC_L_old = 0;
    double detLp/*,r , dX ,dY,dL*/;

    //エンコーダーの位置での前回からの移動距離dL_R,dL_Lを算出
    double dL_L = (double)(ENC_L-ENC_L_old)*(-DIST_PER_PULSE);
    double dL_R = (double)(ENC_R-ENC_R_old)*  DIST_PER_PULSE;

    //角速度SIGMAを算出
    double POS_SIGMA = (dL_R - dL_L)/WHEEL_DIST;
    double dL = (dL_R + dL_L)*0.50000;

    //移動距離deLpを算出
    if(fabs(POS_SIGMA)<0.0001){//左右の速度が等しく直進するとき
        POS_SIGMA = 0.0;
        detLp = dL_R;
    }else{                       //左右どちらかにマシンの向きが変わってるとき
        // r = dL/POS_SIGMA;
        detLp = 2.0000*(dL/POS_SIGMA)*sin(POS_SIGMA*0.50000);
    }

/*    mchn_pose.vel_odom.x = detLp * cos(mchn_pose.pos_odom.theta + (POS_SIGMA/2.0));
    mchn_pose.vel_odom.y = detLp * sin(mchn_pose.pos_odom.theta + (POS_SIGMA/2.0));*/
    double dX = detLp * cos(mchn_pose.pos_odom.theta + (POS_SIGMA/2.0));//X,Yの前回からの移動量計算
    double dY = detLp * sin(mchn_pose.pos_odom.theta + (POS_SIGMA/2.0));
    ENC_R_old = ENC_R;//前回のエンコーダーの値を記録
    ENC_L_old = ENC_L;
    mchn_pose.pos_odom.theta = (ENC_R_old - (-ENC_L_old))*DIST_PER_PULSE/WHEEL_DIST;//現在の角度を算出

    mchn_pose.pos_odom.theta = nomalizeAng(mchn_pose.pos_odom.theta);

    mchn_pose.pos_odom.x += dX;
    mchn_pose.pos_odom.y += dY;
    // mchn_pose.pos_odom.theta = POS_ANG;
    mchn_pose.vel_odom.x =  dX * ROS_RATE;
    mchn_pose.vel_odom.y =  dY * ROS_RATE;
    mchn_pose.vel_odom.theta = (dL_R - (-dL_L))/WHEEL_DIST;
    return ;
}

/*------------------------------------
 * send command to mbed in wheel chair
 * argument:
 *   arg_speed: forward speed   [mm/sec]
 *   arg_theta: CCW turn speed  [radian/sec]
 * ----------------------------------*/
void spinWheel(/*double arg_speed, double arg_theta*/){
    double arg_speed = mchn_pose.tgtTwist.linear.x;
    double arg_theta = mchn_pose.tgtTwist.angular.z;
    ROS_INFO("X:%4.2f   Theta:%4.2f",arg_speed,arg_theta);
    double val_L = -Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST/2)*arg_theta);
    double val_R =  Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST/2)*arg_theta);
    val_L = (int)Limit(val_L,(double)SPEED_MAX,(double)-SPEED_MAX);
    val_R = (int)Limit(val_R,(double)SPEED_MAX,(double)-SPEED_MAX);
    //ROS_INFO("val_L:%2.f   val_R:%2.f",val_L,val_R);

    string cmd_L = boost::lexical_cast<string>(val_L);
    string cmd_R = boost::lexical_cast<string>(val_R);

    string message = "@SS1," + cmd_L + "@SS2," + cmd_R;
    string reply;
    client_socket << message;
    //client_socket >> reply;
    //cout << "Response:" << reply << "\n";
}

// void receiveGoalPose(const geometry_msgs::Pose2D::ConstPtr& cmd_pose){
//     mchn_pose.tgtPose = *cmd_pose;
// }

bool receiveGoalPose(   tms_msg_rc::rc_robot_control::Request &req,
                        tms_msg_rc::rc_robot_control::Response &res){
    if(1 != req.unit){      //Is not vicle unit
        res.result = 0;//SRV_UNIT_ERR;
        return true;
    }
    if(15 != req.cmd){
        res.result = 0;//SRV_CMD_ERR;
        return true;
    }
    mchn_pose.tgtPose.x = req.arg[0];
    mchn_pose.tgtPose.y = req.arg[1];
    mchn_pose.tgtPose.theta = Deg2Rad(req.arg[2]);
    while(! mchn_pose.goPose()){
        ROS_INFO("doing goPose");
    }
}

void receiveCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel){
    mchn_pose.tgtTwist = *cmd_vel;
    // spinWheel(/*cmd_vel->linear.x,cmd_vel->angular.z*/);
}

void receiveJoy(const sensor_msgs::Joy::ConstPtr& joy){
    ROS_INFO("Rrecieve joy");
    mchn_pose.tgtTwist.linear.x =   joy->axes[1]*300;//600;
    mchn_pose.tgtTwist.angular.z =  joy->axes[3]*0.7;//1;
}

bool MachinePose_s::goPose(/*const geometry_msgs::Pose2D::ConstPtr& cmd_pose*/){
    //original PID feedback on error of Angle and Distance
    const double KPang  = 1.0;
    const double KDang  = 0;
    const double KPdist = 2.0;
    const double KDdist = 0;
    bool ret = false;

    double errorX = this->tgtPose.x - this->pos_vicon.x;
    double errorY = this->tgtPose.y - this->pos_vicon.y;
    double targetT = atan2(errorY,errorX);
 
    double theta = this->pos_vicon.theta;
    double errorNX = errorX * cos(-theta) -  errorY * sin(-theta);
    //double errorNY = errorX * sin(-theta) +  errorY * cos(-theta); 
    double errorNT = nomalizeAng(targetT - theta);


   if(this->tgtPose.x==0.0 && this->tgtPose.y==0.0){  //mokutekiti
        errorNX /*= errorNY */= errorNT =  0.0;
    }
    double tmp_spd  = KPdist * errorNX;
    double tmp_turn = KPang * Rad2Deg(errorNT);
    tmp_spd =  Limit(tmp_spd,100,-100);
    tmp_turn = Limit(tmp_turn,30,-30);
    double distance = sqrt(sqr(errorX)+sqr(errorY));
    if (distance <= 200){
        tmp_spd = 0.0;
    }
    printf("spd:%+8.2lf turn:%+4.1lf",tmp_spd,tmp_turn);
    this->tgtTwist.linear.x = tmp_spd;
    this->tgtTwist.angular.z= Deg2Rad(tmp_turn);
    if(distance<=200 && 20>fabs(Rad2Deg(errorNT))){
        ret = true;
    }
    return ret;
}


int main(int argc, char **argv){
    ROS_INFO("wc_controller");
    ros::init(argc, argv, "wc_controller");
    ros::NodeHandle n;

    int Kp_,Ki_,Kd_;
    string s_Kp_,s_Ki_,s_Kd_;
    ros::NodeHandle nh_param("~");
    nh_param.param<int>("Kp",Kp_,4800);
    nh_param.param<int>("Ki",Ki_,/*30*/100);
    nh_param.param<int>("Kd",Kd_,40000);
    s_Kp_ = boost::lexical_cast<string>(Kp_);
    s_Ki_ = boost::lexical_cast<string>(Ki_);
    s_Kd_ = boost::lexical_cast<string>(Kd_);


    try{
        //ClientSocket client_socket ( "192.168.11.99", 4321 );
        string reply;
        try{
            //koyuusinndou 3Hz at Kp = 8000
            //client_socket << "@CR1@CR2@SM1,1@SM2,1@PP1,50@PP2,50@PI1,100@PI2,100@PD1,10@PD2,10";
            client_socket << "@CR1@CR2@SM1,1@SM2,1@PP1,"+s_Kp_+"@PP2,"+s_Kp_+"@PI1,"+s_Ki_+"@PI2,"+s_Ki_+"@PD1,"+s_Kd_+"@PD2,"+s_Kd_;
            client_socket >> reply;
        }
        catch ( SocketException& ) {}
        cout << "Response:" << reply << "\n";;
    }catch ( SocketException& e ){
        cout << "Exception was caught:" << e.description() << "\n";
    }

    db_client = n.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
//    ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, receiveCmdVel);
//    ros::Subscriber cmd_vel_sub = n.subscribe<sensor_msgs::Joy>("/joy", 1, receiveJoy);
//    ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Pose2D>("/mkun_goal_pose", 1, receiveGoalPose);
    ros::ServiceServer service = n.advertiseService("mkun_goal_pose",receiveGoalPose);
    ros::Time current_time, last_time;
    current_time    = ros::Time::now();
    last_time       = ros::Time::now();

    ros::Rate   r(ROS_RATE);
    while(n.ok()){
        // ROS_INFO("");
        spinWheel(/*joy_cmd_spd,joy_cmd_turn*/);
        mchn_pose.updateVicon();
        mchn_pose.goPose();
        //mchn_pose.updateOdom();
        ROS_INFO("x:%4.2lf y:%4.2lf th:%4.2lf",
            mchn_pose.pos_vicon.x,
            mchn_pose.pos_vicon.y,
            mchn_pose.pos_vicon.theta);
/*        ROS_INFO("x:%4.2lf y:%4.2lf th:%4.2lf",
            mchn_pose.pos_odom.x,
            mchn_pose.pos_odom.y,
            Rad2Deg(mchn_pose.pos_odom.theta));*/
        last_time = current_time;
        current_time = ros::Time::now();
        ros::spinOnce();
        r.sleep();
    }
    return(0);
}

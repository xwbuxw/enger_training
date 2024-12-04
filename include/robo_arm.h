#ifndef ROBO_ARM__H
#define ROBO_ARM__H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <nav_msgs/Odometry.h>

#define DEG_TO_RAD(angleInDegrees) ((angleInDegrees) * PI / 180.0)
#define RAD_TO_DEG(angleInRadians) ((angleInRadians) * 180.0 / PI)

#define RED_IN_CAR (struct arm_pose){1, 1, 1, 1, 1}
#define GREEN_IN_CAR (struct arm_pose){1, 1, 1, 1, 1}
#define BULE_IN_CAR (struct arm_pose){1, 1, 1, 1, 1}
#define CIRCULAR_RELATIVE_CAR (struct arm_pose){1, 1, 1, 1, 1}//没有用

#define INIT_POSE_X 0
#define INIT_POSE_Y 0
#define INIT_POSE_Z 0
#define INIT_POSE_PAW 0
#define INIT_POSE_CAM 0


#define RED_X 99.5 //红色方块在车上的x坐标
#define RED_Y 106
#define CAM_ANGLE_RED 2.355 
#define GREEN_X 99.5
#define GREEN_Y 199
#define CAM_ANGLE_GREEN 2.355
#define BULE_X -25
#define BULE_Y 199-30
#define CAM_ANGLE_BLUE 3.1415
#define CAR_HIGHT 0.2 //快在车上的高度
#define TEKEUP_HEIGHT 0.5 //从车上拿起的高度

#define CIRCULAR_RED_X -60 //圆相对与车的x坐标
#define CIRCULAR_RED_Y -180
#define CIRCULAR_RED_CAM_ANGLE 3.1415 + 0.3
#define CIRCULAR_GREEN_X -210
#define CIRCULAR_GREEN_Y -180
#define CIRCULAR_GREEN_CAM_ANGLE 3.1415
#define CIRCULAR_BLUE_X -360
#define CIRCULAR_BLUE_Y -180
#define CIRCULAR_BLUE_CAM_ANGLE 3.1415+0.3
#define CIRCULAR_HEIGHT 0.5 //圆在地上的高度 




#define PAW_OPEN 0.5  //家爪打开的角度
#define PAW_CLOSE 0.5



#define CAM_ANGLE 0.1    //nouseful

#define LIMIT_VISION_X 0.2 //视觉对准
#define LIMIT_VISION_Y 0.2 //视觉对准

#define MOVE_LITTLE_DISTANCE 0.5 //对准阶段一次移动的距离


struct arm_pose{
        double x;
        double y;
        double z;
        double cam_angle;
        double paw_angle;
        double temp_time; //must be delete
    };
struct vision_data{
    double x;
    double y;
    double color;
    };


extern std::vector<arm_pose> arm_control;
extern arm_pose red_circle;
extern arm_pose blue_circle;
extern arm_pose green_circle;


class RobotArm {
public:

    ros::NodeHandle nh;
    ros::Publisher arm_pub;
    ros::Subscriber arm_sub;
    ros::Subscriber vision_sub;

    RobotArm(){
        arm_pub = nh.advertise<nav_msgs::Odometry>("/arm_ctrl", 10);
        arm_sub = nh.subscribe<nav_msgs::Odometry>("/arm_read",10, &RobotArm::arm_pose_cb,this);
        vision_sub = nh.subscribe<std_msgs::ColorRGBA>("/vision_data",10, &RobotArm::vision_cb,this);
        arm_pose init_pose = { //TODO ���޸ģ���kxdԼ����ʼλ��
        .x = INIT_POSE_X,
        .y = INIT_POSE_Y,
        .z = INIT_POSE_Z,
        .cam_angle = INIT_POSE_CAM,
        .paw_angle = INIT_POSE_PAW
        };
        arm_control.push_back(init_pose);
    }


    void choose (char color);
    void put_down(char color);
    void test();
    int vision_correction(char color);

private:

    vision_data vision_data_;
    int add_arm_pose(float x, float y, float z, float cam_angle, float paw_angle);
    int vision(char color);
    void movelittle(char direction);
    void arm_pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void vision_cb(const std_msgs::ColorRGBA::ConstPtr& msg);
    bool arm_arrived(arm_pose target_arm_pose);
    void arm_pose_pub(int index);
    int add_arm_pose_from_define(arm_pose pose);


};

    



#endif

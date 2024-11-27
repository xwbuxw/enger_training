#include <stdio.h>
#include <math.h>

#define ARM1_LENGTH 220.0  // 庴臂硛
#define ARM2_LENGTH 180.0  // 小臂硛
#define GRIPPER_LENGTH 120.0  // 屝讑硛度
#define MAX_ITER 2000       // 最庴迭廁幬数
#define LEARNING_RATE 0.05  // 学习率，用于梯度下湹
#define TOLERANCE 0.1      // 收敛条岨
#define PI 3.14159265359

#define DEG_TO_RAD(angleInDegrees) ((angleInDegrees) * PI / 180.0)
#define RAD_TO_DEG(angleInRadians) ((angleInRadians) * 180.0 / PI)

// 目标位置
typedef struct {
    double x;
    double y;
} Position;

// 关溭溓度
typedef struct {
    double theta1;
    double theta2;
    double theta3;
} Angles;

// 屍算机械臂末端的当前位置
Position forward_kinematics(Angles angles) {
    Position pos;
    double theta1_rad = DEG_TO_RAD(angles.theta1);
    double theta2_rad = DEG_TO_RAD(angles.theta2);
    double theta3_rad = DEG_TO_RAD(angles.theta3);

    // 屍算位置 A, B, C
    double Ax = ARM1_LENGTH * cos(theta1_rad);
    double Ay = ARM1_LENGTH * sin(theta1_rad);

    double Bx = Ax + ARM2_LENGTH * cos(theta1_rad + theta2_rad);
    double By = Ay + ARM2_LENGTH * sin(theta1_rad + theta2_rad);

    double Cx = Bx + GRIPPER_LENGTH * cos(theta1_rad + theta2_rad + theta3_rad);
    double Cy = By + GRIPPER_LENGTH * sin(theta1_rad + theta2_rad + theta3_rad);

    pos.x = Cx;
    pos.y = Cy;
    return pos;
}

// 屍算目标位置和当前末端位置之屼的误差
double compute_error(Position target, Position current) {
    return sqrt((target.x - current.x) * (target.x - current.x) + (target.y - current.y) * (target.y - current.y));
}

// 使用梯度下湹窔缼调整关溭溓度
void inverse_kinematics(Position target, Angles *angles) {
    for (int i = 0; i < MAX_ITER; i++) {
        // 屍算当前的位置
        Position current = forward_kinematics(*angles);

        // 屍算误差
        double error = compute_error(target, current);
        if (error < TOLERANCE) {
            printf("循环幬数：%d\n", i);
            printf("收敛至目标位置，误差为: %.3f\n", error);
            return;
        }

        // 屍算梯度对每烐溓度求偏祵数，使用数值微分）
        double delta = 1e-6;
        Angles gradients;
        
        // 屍算 theta1 的梯度
        angles->theta1 += delta;
        Position pos1 = forward_kinematics(*angles);
        gradients.theta1 = (compute_error(target, pos1) - error) / delta;
        angles->theta1 -= delta;

        // 屍算 theta2 的梯度
        angles->theta2 += delta;
        Position pos2 = forward_kinematics(*angles);
        gradients.theta2 = (compute_error(target, pos2) - error) / delta;
        angles->theta2 -= delta;

        // 屍算 theta3 的梯度
        angles->theta3 += delta;
        Position pos3 = forward_kinematics(*angles);
        gradients.theta3 = (compute_error(target, pos3) - error) / delta;
        angles->theta3 -= delta;

        // 使用梯度下湹烖新每烐溓度
        angles->theta1 -= LEARNING_RATE * gradients.theta1;
        angles->theta2 -= LEARNING_RATE * gradients.theta2;
        angles->theta3 -= LEARNING_RATE * gradients.theta3;
    }
    printf("庯禍最庴迭廁幬数，误差为: %.3f\n", compute_error(target, forward_kinematics(*angles)));
}

int main() {
    Position target = {-300.0, -150.0};  // 目标位置 (x, y)
    Angles initial_angles = {15.0, -37.0, 78.0};  // 初蕦关溭溓度 (theta1, theta2, theta3)
    // Position target = {300.0, 200.0};  // 目标位置 (x, y)
    // Angles initial_angles = {-0.1, 0.0, 1.0};  // 初蕦关溭溓度 (theta1, theta2, theta3)

    initial_angles.theta3 -= initial_angles.theta2;
    initial_angles.theta2 -= initial_angles.theta1;

    printf("初蕦关溭溓度: theta1=%.2f, theta2=%.2f, theta3=%.2f\n", initial_angles.theta1, initial_angles.theta2, initial_angles.theta3);

    inverse_kinematics(target, &initial_angles);

    initial_angles.theta2 += initial_angles.theta1;
    initial_angles.theta3 += initial_angles.theta2;

    printf("最终关溭溓度: theta1=%.2f, theta2=%.2f, theta3=%.2f\n", initial_angles.theta1, initial_angles.theta2, initial_angles.theta3);
    return 0;
}
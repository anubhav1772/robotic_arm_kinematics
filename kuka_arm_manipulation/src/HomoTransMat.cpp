#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <ros/ros.h>

#define PI 3.14159265

using Eigen::Matrix4d;

using std::cout;
using std::endl;
using std::sin;
using std::cos;

/**
 * Homogeneous Transformation Matrix 
 * 
 * @param theta angle between x(n-1) and x(n) measured about z(n-1)
 * @param d distance along z(n-1) from origin of (n-1)th frame to intersection 
 *          of x(n) and z(n-1) axes
 * @param a distance along x(n) from origin of (n)th frame to intersection
 *          of x(n) and z(n-1) axes
 * @param alpha angle between z(n-1) and z(n) measured about x(n) 
 * @return Homogeneous transformation matrix from (n-1)th frame to nth frame based 
 *         on row of DH parameter table
 */
Matrix4d tf_mat_n_minus_1_to_n(double theta, double d, double a, double alpha)
{
    Matrix4d T;
    T << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    
    return T;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trans_mat");
    ros::NodeHandle nh;

    Matrix4d T0_7, T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T_Z180, T_Y90;
    double theta1, theta2, theta3, theta4, theta5, theta6;

    theta1 = 0;
    theta2 = 0;
    theta3 = 0;
    theta4 = 0;
    theta5 = 0;
    theta6 = 0;

    T0_1 = tf_mat_n_minus_1_to_n(theta1, 0.75, 0, 0);
    T1_2 = tf_mat_n_minus_1_to_n(theta2 - PI/2, 0, 0.35, - PI/2);
    T2_3 = tf_mat_n_minus_1_to_n(theta3, 0, 1.25, 0);
    T3_4 = tf_mat_n_minus_1_to_n(theta4, 1.5, -0.054, -PI/2);
    T4_5 = tf_mat_n_minus_1_to_n(theta5, 0, 0, PI/2);
    T5_6 = tf_mat_n_minus_1_to_n(theta6, 0, 0, -PI/2);
    T6_7 = tf_mat_n_minus_1_to_n(0, 0.303, 0, 0);

    T0_7 = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7;
    
    cout << T0_7 << endl;

    // Intrinsic rotations (pre-multiply)
    // 1. Rotation by 180 deg about Z-axis
    // 2. Rotation by -90 deg about Y-axis
    T_Z180 << -1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    T_Y90 << 0, 0, -1, 0,
                0, 1, 0, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;

    cout << T_Y90*T_Z180*T0_7 << endl;
    
    return 0;
}
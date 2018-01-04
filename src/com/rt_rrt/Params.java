package com.rt_rrt;

public class Params {
    // 收敛距离
    public static double converge = 0.3;
    // 采样概率。。。
    public static double alpha = 0.1;
    public static double beta = 1.4;
    // neighbour
    public static double rrtstarradius = 0.7;
    // stepsize
    public static double epsilon = 0.5;
    // 最大邻居数目
    public static double maxNeighbours = 50;
    // 用来rewire的限时
    public static double allowedTimeRewiring = 500;
    // 传感器扫描半径
    public static double sensorRadius = 4;
    // 机器人移动速度
    public static double robotSpeed = 0.2;
    // 点靠近误差
    public static double nearError = 0.02;
    // 移动障碍物旋转速度
    public static double dyanamicRotationSpeed = 0.1;
}

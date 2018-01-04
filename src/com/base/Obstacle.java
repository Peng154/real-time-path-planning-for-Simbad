package com.base;

import javax.vecmath.Vector2d;

public abstract class Obstacle {

    protected double safe_radius; // 防止移动机器人碰撞的安全半径

    public Obstacle(double safe_radius){
        this.safe_radius = safe_radius;
    }

    public static double getEuclideanDistance(Vector2d source, Vector2d dest){
        Vector2d temp = new Vector2d();
        temp.sub(dest,source);
        return temp.length();
    }

    /**
     * 检测直线是否会和障碍物存在交点
     * @param s
     * @param e
     * @return
     */
    public abstract boolean checkLineCrossObstacle(Vector2d s, Vector2d e);

    /**
     * 检测点是否在障碍物内
     * @param pos
     * @return
     */
    public abstract boolean checkPosInObstacle(Vector2d pos);

}

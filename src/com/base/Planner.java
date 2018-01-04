package com.base;

import javax.vecmath.Vector2d;
import java.util.List;

public abstract class Planner {

    public List<Obstacle> obstacles;
    public Vector2d startPos;
    public Vector2d endPos;
    public List<Double> boundaries;

    /**
     * 路径规划器的基本元素是：已知障碍物、起始点、终点、规划边界
     * @param obstacles 障碍物的点集列表，每个障碍物的点按照顺时针方向排列,左上角为第一个点（目前的障碍物都是矩形并且不旋转）
     * @param startPos
     * @param endPos
     * @param boundaries
     */
    public Planner(List<Obstacle> obstacles, Vector2d startPos, Vector2d endPos, List<Double> boundaries){
        this.obstacles = obstacles;
        this.startPos = startPos;
        this.endPos = endPos;
        this.boundaries = boundaries;
    }

    public List<Obstacle> getObstacles() {
        return obstacles;
    }

    public Vector2d getStartPos() {
        return startPos;
    }

    public Vector2d getEndPos() {
        return endPos;
    }

    public List<Double> getBoundaries() {
        return boundaries;
    }

    /**
     * 开始规划
     * @return
     */
    public abstract List<Vector2d> plan();

    /**
     * 获取最佳路径
     * @return
     */
    public abstract List<Vector2d> getBestPath();


}

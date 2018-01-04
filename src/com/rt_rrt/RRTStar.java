package com.rt_rrt;

import com.base.Obstacle;
import com.rrt.RRT;
import com.rrt.RRTNode;

import javax.vecmath.Vector2d;
import java.util.LinkedList;
import java.util.List;

public class RRTStar extends SMP{
    /**
     * 路径规划器的基本元素是：已知障碍物、起始点、终点、规划边界
     *
     * @param obstacles  障碍物的点集列表，每个障碍物的点按照顺时针方向排列,左上角为第一个点（目前的障碍物都是矩形并且不旋转）
     * @param startPos
     * @param endPos
     * @param boundaries
     */
    public RRTStar(List<Obstacle> obstacles, Vector2d startPos, Vector2d endPos, List<Double> boundaries) {
        super(obstacles, startPos, endPos, boundaries);
    }

    protected List<RRTNode> findClosestNeighbours(RRTNode node){
        List<RRTNode> neighbours = new LinkedList<>();
        for(RRTNode n:rrt.getRrtTree()){
            double dist = getEuclideanDistance(n, node);
            if(dist < Params.rrtstarradius && dist>0.0001){
                neighbours.add(n);
            }
        }
        return neighbours;
    }
}

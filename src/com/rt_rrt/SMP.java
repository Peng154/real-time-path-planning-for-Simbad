package com.rt_rrt;

import com.base.Obstacle;
import com.base.Planner;
import com.rrt.RRT;
import com.rrt.RRTNode;

import javax.vecmath.Vector2d;
import java.util.List;

public class SMP extends Planner{

    public static boolean goalFound;
    public static boolean sampledInGoalRegion;
    public static boolean moveNow;

    // 根节点
    public static RRTNode root;
    // 路径规划目标点
    public static RRTNode target;
    protected RRT rrt;

    /**
     * 路径规划器的基本元素是：已知障碍物、起始点、终点、规划边界
     *
     * @param obstacles  障碍物的点集列表，每个障碍物的点按照顺时针方向排列,左上角为第一个点（目前的障碍物都是矩形并且不旋转）
     * @param startPos
     * @param endPos
     * @param boundaries
     */
    public SMP(List<Obstacle> obstacles, Vector2d startPos, Vector2d endPos, List<Double> boundaries) {
        super(obstacles, startPos, endPos, boundaries);
        rrt = new RRT(startPos.x, startPos.y);
        goalFound = false;
        sampledInGoalRegion = false;
        moveNow = false;
        target = null;
        root = rrt.getNode(0);
    }

//    public void addNode(RRTNode node){
//        rrt.getRrtTree().add(node);
//        if(getEuclideanDistance(node.pos, endPos) < Params.converge){
//            goalFound = true;
//            sampledInGoalRegion = true;
//            target = node;
//        }
//    }

    protected RRTNode sample(){
        RRTNode node = new RRTNode();
        node.pos.x = rand(boundaries.get(0),boundaries.get(1));
        node.pos.y = rand(boundaries.get(2),boundaries.get(3));
        return node;
    }



    protected double rand(double a, double b){
        assert b>a;
        double r = Math.random();
        return a+r*(b-a);
    }

    protected boolean checkSample(RRTNode n){
        for(Obstacle o:obstacles){
            if(o.checkPosInObstacle(n.pos))
                return false;
        }
        return true;
    }

    protected boolean checkCollision(RRTNode s, RRTNode e){
        for(Obstacle o:obstacles){
            if(o.checkLineCrossObstacle(s.pos, e.pos))
                return false;
        }
        return true;
    }


    @Override
    public List<Vector2d> plan() {
        return null;
    }

    @Override
    public List<Vector2d> getBestPath() {
        return null;
    }

    protected double getEuclideanDistance(Vector2d source, Vector2d dest){
        Vector2d temp = new Vector2d();
        temp.sub(dest,source);
        return temp.length();
    }

    protected double getEuclideanDistance(RRTNode source, RRTNode dest){
        return getEuclideanDistance(source.pos, dest.pos);
    }
}

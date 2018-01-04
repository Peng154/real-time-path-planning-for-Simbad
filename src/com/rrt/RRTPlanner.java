package com.rrt;

import com.base.Obstacle;
import com.base.Planner;

import javax.vecmath.Vector2d;
import java.util.ArrayList;
import java.util.List;

public class RRTPlanner extends Planner {
    private double STEP_SZIE = 0.3;
    private double SAMPLE_SIZE = -1;
    private int PATHS_LIMIT = 1;
    private int MAX_ITERATIONS = 500000;

    private RRT rrt;
    private List<List<Integer>> paths;

    private double getEuclideanDistance(Vector2d source, Vector2d dest){
        Vector2d temp = new Vector2d();
        temp.sub(dest,source);
        return temp.length();
    }

    /**
     *
     * @param obstacles 障碍物的点集列表，每个障碍物的店按照顺时针方向排列,左上角为第一个点
     * @param startPos 初始点
     * @param endPos 目标点
     * @param boundaries 运动边界(Xmin, Xmax, Ymin, Ymax)暂定为正方形
     */
    public RRTPlanner(List<Obstacle> obstacles, Vector2d startPos, Vector2d endPos, List<Double> boundaries, int path_limits, double stepSize){
        super(obstacles, startPos, endPos, boundaries);
        // 记得处理一下（扩大）边界和障碍物
        this.obstacles = obstacles;
        this.startPos = startPos;
        this.endPos = endPos;
        this.boundaries = boundaries;
        this.paths = new ArrayList<>();
        this.PATHS_LIMIT = path_limits;
        this.SAMPLE_SIZE = boundaries.get(1);
        this.STEP_SZIE = stepSize;
        rrt = new RRT(startPos.x, startPos.y);
    }


    boolean checkNodeToGoal(Vector2d goalPos, Vector2d nodePos){
        return goalPos.epsilonEquals(nodePos, STEP_SZIE/2);
    }

    boolean checkOutsideObstacles(Vector2d pos){
        for(Obstacle obstacle:obstacles){
            if(obstacle.checkPosInObstacle(pos)){
                return false;
            }
        }
        return true;
    }

    boolean checkInsideBoundary(Vector2d pos){
        if(pos.x < boundaries.get(0) || pos.x > boundaries.get(1) || pos.y < boundaries.get(2) || pos.y > boundaries.get(3)){
            return false;
        }
        else {
            return true;
        }
    }

    RRTNode generateNode(){
        RRTNode node = new RRTNode();
        node.pos.x = SAMPLE_SIZE * (Math.random()-0.5) * 2;
        node.pos.y = SAMPLE_SIZE * (Math.random()-0.5) * 2;
        return node;
    }

    boolean addNewNodeToRRT(RRTNode newNode){
        boolean success = false;
        // 找出最近的点
        int nearestNodeID = rrt.getNearestNodeID(newNode.pos);
        RRTNode nearestNode = rrt.getNode(nearestNodeID);
        double theta = Math.atan2(newNode.pos.y - nearestNode.pos.y, newNode.pos.x - nearestNode.pos.x); // 求角度

        // 根据步长改变新点的位置（所有节点之间距离固定。。。。）
        newNode.pos.x = nearestNode.pos.x + STEP_SZIE * Math.cos(theta);
        newNode.pos.y = nearestNode.pos.y + STEP_SZIE * Math.sin(theta);

        if(checkInsideBoundary(newNode.pos) && checkOutsideObstacles(newNode.pos)){
            // 添加新节点到rrt树中
            rrt.addNode(newNode, nearestNodeID);
            success = true;
        }

        return success;
    }

    @Override
    public List<Vector2d> plan(){
        int iter = 0;
        paths.clear(); // 清空旧的结果
        boolean addNodeResult = false;
        boolean nodeToGoal = false;
        List<Integer> path = null;
        List<Vector2d> result = new ArrayList<>();

        while (iter < MAX_ITERATIONS){
            // 路径解的数量还不够
            if(paths.size() < PATHS_LIMIT){
                RRTNode node  =generateNode();
                addNodeResult = addNewNodeToRRT(node);
                if(addNodeResult){
                    nodeToGoal = checkNodeToGoal(endPos, node.pos);
                    // 如果新的点到达了终点
                    if(nodeToGoal){
                        path = rrt.getRootToEndPath(node.nodeID);
                        paths.add(path);
                        System.out.println("找到新路径，路径总数："+ paths.size()+", iter:"+iter+ ", size:"+path.size());
                    }
                }
            }
            else {
                break; // 找到了足够数量的路径，返回最优解
            }
            iter ++;
        }

        result = getBestPath();
        return result;
    }

    @Override
    public List<Vector2d> getBestPath(){
        List<Vector2d> result = new ArrayList<>();
        List<Integer> path = new ArrayList<>();

        if(paths.size() == 0){
            result = null; // 没有找到解
        }
        int minSize = 999999;
        // 求出最优路径
        for (List<Integer> p: paths) {
            if(p.size() < minSize){
                path = p;
                // 因为每个节点之间的距离都是固定的stepSize，所以可以用size来代替
                minSize = p.size();
            }
        }

        for(int i:path){
            result.add(rrt.getNode(i).pos);
        }

        return result;
    }

}

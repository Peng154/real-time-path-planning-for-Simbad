package com.rrt_star;

import com.base.Obstacle;
import com.base.Planner;
import com.rrt.RRT;
import com.rrt.RRTNode;

import javax.vecmath.Vector2d;
import java.util.ArrayList;
import java.util.List;

public class RRTStarPlanner extends Planner {
    private double STEP_SZIE = 0.3; // 扩展步长
    private double NEIGHBOURHOOD = 0.5; // 邻居范围
    private double SAMPLE_SIZE = -1; // 采样范围
    private int PATHS_LIMIT = 1; // 路径解数量
    private int MAX_ITERATIONS = 200000; // 最大迭代次数

    private RRT rrt;
    private List<Integer> nodesAroundGoal;

    private double getEuclideanDistance(Vector2d source, Vector2d dest){
        Vector2d temp = new Vector2d();
        temp.sub(dest,source);
        return temp.length();
    }

    /**
     *
     * @param obstacles 障碍物的点集列表，每个障碍物的点按照顺时针方向排列,左上角为第一个点（目前的障碍物都是矩形并且不旋转）
     * @param startPos 初始点
     * @param endPos 目标点
     * @param boundaries 运动边界(Xmin, Xmax, Ymin, Ymax)暂定为正方形，也就是说Xmax=Ymax，坐标原点在框架中心点
     */
    public RRTStarPlanner(List<Obstacle> obstacles, Vector2d startPos, Vector2d endPos, List<Double> boundaries,
                          int path_limits, double stepSize){

        super(obstacles, startPos, endPos, boundaries);
        // 记得处理一下（扩大）边界和障碍物
        this.obstacles = obstacles;
        this.startPos = startPos;
        this.endPos = endPos;
        this.boundaries = boundaries;
        this.nodesAroundGoal = new ArrayList<>();
        this.PATHS_LIMIT = path_limits;
        this.SAMPLE_SIZE = boundaries.get(1);
        this.STEP_SZIE = stepSize;
        this.NEIGHBOURHOOD = 1.3 * stepSize; // 邻居距离为1.2倍stepSize
        rrt = new RRT(startPos.x, startPos.y);
    }

    /**
     * 检测是否到达终点
     * @param goalPos
     * @param nodePos
     * @return
     */
    private boolean checkNodeToGoal(Vector2d goalPos, Vector2d nodePos){
        // 认为在终点半径0.5倍stepSize范围内都是到达了终点
        return goalPos.epsilonEquals(nodePos, STEP_SZIE/2);
    }

    /**
     * 检查点是否在凸多边形障碍物内
     * @param pos
     * @return
     */
    private boolean checkOutsideObstacles(Vector2d pos){
        for(Obstacle obstacle:obstacles){
            if(obstacle.checkPosInObstacle(pos)){
                return false;
            }
        }
        return true;
    }

    /**
     * 检查点是否在框架范围内
     * @param pos
     * @return
     */
    private boolean checkInsideBoundary(Vector2d pos){
        if(pos.x < boundaries.get(0) || pos.x > boundaries.get(1) || pos.y < boundaries.get(2) || pos.y > boundaries.get(3)){
            return false;
        }
        else {
            return true;
        }
    }

    /**
     * 判断线段是否在障碍物外面
     * 因为保证了线段的点都在障碍物外面，所以只需要判断线段是否和障碍物相交
     * @param s
     * @param e
     * @return
     */
    private boolean checkLineOutsideObstacles(Vector2d s, Vector2d e){
        for(Obstacle o:obstacles){
            if(o.checkLineCrossObstacle(s,e)){
                return false;
            }
        }
        // 直线和所有的障碍物的边都不相交
        return true;
    }

    /**
     * 采样范围内生成新的点
     * @return
     */
    private RRTNode generateNode(){
        RRTNode node = new RRTNode();
        node.pos.x = SAMPLE_SIZE * (Math.random()-0.5) * 2;
        node.pos.y = SAMPLE_SIZE * (Math.random()-0.5) * 2;
        return node;
    }

    private double getPathCost(int endNodeID){
        double cost = 0;
        List<Integer> path = rrt.getRootToEndPath(endNodeID);
        for(int i=1;i<path.size();++i){
            cost+=getEuclideanDistance(rrt.getNode(path.get(i)).pos,rrt.getNode(path.get(i-1)).pos);
        }
        return cost;
    }
    /**
     * 从给出的点中找出到root对短的点(这里的计算有点问题，因为点之间的连线不一定定长)
     * @param nodes
     * @return
     */
    private int getShortestPathNode(List<Integer> nodes){
        double len = getPathCost(nodes.get(0));
        int node = nodes.get(0);
        for(int i = 1;i<nodes.size();++i){
            double temp = getPathCost(nodes.get(i));
            if(temp < len){
                len = temp;
                node = nodes.get(i);
            }
        }
        return node;
    }

    /**
     * 根据新的点重新连接他的邻居节点
     * @param newNodeID
     * @param neighbors
     */
    private void rewireNodes(int newNodeID, List<Integer> neighbors){
        RRTNode newNode = rrt.getNode(newNodeID);
        double lenToNewNode = getPathCost(newNodeID);
        for(int node: neighbors){
            double lenToNeighbour = getPathCost(node);
            double dis = getEuclideanDistance(rrt.getNode(node).pos, newNode.pos);
            // 新的路径要更短而且不穿过障碍物
            if(lenToNeighbour > (lenToNewNode + dis)){
                // 移除旧的父子节点关系
                RRTNode oldFatherNode = rrt.getParent(node);
                oldFatherNode.children.remove(new Integer(node));
                // 添加新的父子节点关系
                rrt.setParent(node, newNodeID);
                rrt.addChildID(newNodeID, node);
//                rrt.setCostToRoot(node, lenToNewNode+dis);
            }
        }
    }

    /**
     * 找出某个节点周围可以安全连接的节点
     * @param neighbours
     * @param node
     * @return
     */
    private List<Integer> findSafeNeighbours(List<Integer> neighbours, RRTNode node){
        List<Integer> safeNeighbours = new ArrayList<>();
        for(int i:neighbours){
            if(checkLineOutsideObstacles(rrt.getNode(i).pos, node.pos)){
                safeNeighbours.add(i);
            }
        }
        return safeNeighbours;
    }

    /**
     * 尝试向rrt中添加新的节点
     * @param newNode
     * @return
     */
    private boolean addNewNodeToRRT(RRTNode newNode){
        boolean success = false;
        int nearestNodeID = rrt.getNearestNodeID(newNode.pos);
        RRTNode nearestNode = rrt.getNode(nearestNodeID);

        // 先判断距离
        if(getEuclideanDistance(newNode.pos, nearestNode.pos) > STEP_SZIE){
            double theta = Math.atan2(newNode.pos.y - nearestNode.pos.y, newNode.pos.x - nearestNode.pos.x); // 求角度
            // 修正新节点的位置
            newNode.pos.x = nearestNode.pos.x + STEP_SZIE * Math.cos(theta);
            newNode.pos.y = nearestNode.pos.y + STEP_SZIE * Math.sin(theta);
        }

        if(checkInsideBoundary(newNode.pos) && checkOutsideObstacles(newNode.pos)){
            // 找出邻居节点（优化：可以考虑一下不将新的点加入rrtTree，而是用他的nearestNode优化树的路径！！！控制树的密度）
            List<Integer> neighbours = rrt.getNeighbours(newNode.pos, NEIGHBOURHOOD);
            List<Integer> safeNeighbours = findSafeNeighbours(neighbours, newNode);
            // 只有当新节点的所有邻居是可以到达的，才能继续下一步
            if(safeNeighbours.size() != 0){
//                if(neighbours.size() < K_DENSITY){
                    // 寻找从root到安全邻居节点路径最短的，连接新节点和该邻居节点（加入rrtTree）
                    int fatherID = getShortestPathNode(safeNeighbours);
                    rrt.addNode(newNode, fatherID);
                    // 重新通过新节点连接邻居节点（如果距离会更短）
                    rewireNodes(newNode.nodeID, safeNeighbours);
                    success = true;
//                }
//                else {
//                    rewireNodes(nearestNodeID, safeNeighbours);
//                }
            }
        }
        return success;
    }

    @Override
    public List<Vector2d> plan(){
        int iter = 0;
        nodesAroundGoal.clear(); // 清空旧的结果
        boolean addNodeResult = false;
        boolean nodeToGoal = false;
        List<Vector2d> result = new ArrayList<>();

        while (iter < MAX_ITERATIONS){
            if(nodesAroundGoal.size() < PATHS_LIMIT){
                RRTNode node  =generateNode();
                addNodeResult = addNewNodeToRRT(node);
                if(addNodeResult){
                    nodeToGoal = checkNodeToGoal(endPos, node.pos);
                    if(nodeToGoal){
                        nodesAroundGoal.add(node.nodeID);
                        System.out.println("找到新路径，路径总数："+ nodesAroundGoal.size()+", iter:"+iter);
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

        if(nodesAroundGoal.size() == 0){
            result = null; // 没有找到解
        }
        double minCost = getPathCost(nodesAroundGoal.get(0));
        path = rrt.getRootToEndPath(nodesAroundGoal.get(0));
        for(int i=1;i<nodesAroundGoal.size();++i){
            List<Integer> temp = rrt.getRootToEndPath(nodesAroundGoal.get(i));
            double cost = getPathCost(nodesAroundGoal.get(i));
            if(cost< minCost){
                path = temp;
                minCost = cost;
            }
        }

        for(int i:path){
            result.add(rrt.getNode(i).pos);
        }

        return result;
    }

}

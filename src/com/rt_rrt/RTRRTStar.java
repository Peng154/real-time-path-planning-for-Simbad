package com.rt_rrt;

import com.base.CircleObstacle;
import com.base.Obstacle;
import com.rrt.RRTNode;

import javax.vecmath.Vector2d;
import java.util.*;

public class RTRRTStar extends InformedRRTStar{
    private Queue<RRTNode> rewireRand;
    private Queue<RRTNode> rewireRoot;
    private List<RRTNode> closestNeighbours;

    private List<RRTNode> pushToRewireRoot;
    private double timeKeeper;
    private List<CircleObstacle> moving_obstacles;
    private List<RRTNode> visited_set;
    public Deque<RRTNode> currPath;

    /**
     * 路径规划器的基本元素是：已知障碍物、起始点、终点、规划边界
     *
     * @param obstacles  障碍物的点集列表，每个障碍物的点按照顺时针方向排列,左上角为第一个点（目前的障碍物都是矩形并且不旋转）
     * @param startPos
     * @param endPos
     * @param boundaries
     */
    public RTRRTStar(List<Obstacle> obstacles, Vector2d startPos, Vector2d endPos, List<Double> boundaries, List<CircleObstacle> moving_obstacles) {
        super(obstacles, startPos, endPos, boundaries);
        rewireRand = new LinkedList<>();
        rewireRoot = new LinkedList<>();
        closestNeighbours = new LinkedList<>();
        pushToRewireRoot = new LinkedList<>();
        currPath = new LinkedList<>();
        visited_set = new LinkedList<>();
        // 每次迭代的检测到的移动障碍物
        this.moving_obstacles = moving_obstacles;
    }

    public void nextIter(Robot agent){
        timeKeeper = System.currentTimeMillis();
        expandAndRewire();
        if(goalFound){
            updateBestPath();
        }
//        System.out.println("pathsize"+currPath.size());
//        System.out.println("treesize"+rrt.getTreeSize());
        if(currPath.size() > 1 && getEuclideanDistance(agent.getLocation(), root.pos) < Params.nearError){
            System.out.println("change root");
            currPath.poll(); // 第一个是root
            RRTNode nextPoint =currPath.poll(); // 第二个才是下一目标点
            changeRoot(nextPoint);

            visited_set.clear();
            pushToRewireRoot.clear();
            rewireRoot.clear();
        }
        closestNeighbours.clear();
    }

    public void changeRoot(RRTNode node){
        node.children.add(root.nodeID);
        node.parentID = -1;
        node.costToRoot = 0;

        root.parentID = node.nodeID;
        root.children.remove(new Integer(node.nodeID));
        root.costToRoot = getEuclideanDistance(root, node);
        root = node;
    }

    /**
     * 寻找最优路径
     */
    private void updateBestPath(){
        Deque<RRTNode> updatePath = new LinkedList<>();
        RRTNode pathNode = target;
//        boolean currentPathBlocked = false;
        // 如果找到路径
        if(goalFound){
            do{
                // 有可能动态障碍物处于到终点的路径上面
//                if(pathNode.costToRoot != Double.POSITIVE_INFINITY){
                    currPath.addFirst(pathNode);
                    pathNode = rrt.getNode(pathNode.parentID);
//                }
//                else {
//                    currPath.clear();
//                    currentPathBlocked = true;
//                    break;
//                }
            }while (pathNode != null);
//            if(!currentPathBlocked) return;
            return;
        }

        RRTNode curr_node = root;
        while (!curr_node.children.isEmpty()){
            RRTNode tempNode = rrt.getNode(curr_node.children.get(0));
            double cost_ = cost(tempNode);
            double minCost = cost_ + getHeuristic(tempNode);
            for(int i:curr_node.children){
                RRTNode node = rrt.getNode(i);
                cost_ = cost(node);
                double cost_new = cost_ + getHeuristic(node);
                if(cost_new < minCost){
                    minCost = cost_new;
                    tempNode = node;
                }
            }
            updatePath.add(tempNode);
            if(tempNode.children.isEmpty() || cost(tempNode) == Double.POSITIVE_INFINITY){
                visited_set.add(tempNode);
                break;
            }
            curr_node = tempNode;
        }
        if(currPath.size() == 0){
            currPath.add(root);
        }
        if(getEuclideanDistance(updatePath.getLast().pos, endPos) < getEuclideanDistance(currPath.getLast().pos, endPos)){
            currPath = updatePath;
        }

    }

    /**
     * 获取估计代价
     * @param node
     * @return
     */
    private double getHeuristic(RRTNode node){
        if(visited_set.contains(node)){
            return Double.POSITIVE_INFINITY;
        }
        else {
            return getEuclideanDistance(endPos, node.pos);
        }
    }

    /**
     * 扩展并重连rrt树
     */
    private void expandAndRewire(){
        RRTNode u = this.sample();
        RRTNode v = getClosestNeighbour(u);
        double dist = getEuclideanDistance(u,v);

        if(dist > Params.epsilon){
            double theta = Math.atan2(u.pos.y - v.pos.y, u.pos.x - v.pos.x); // 求角度
            // 修正新节点的位置
            u.pos.x = v.pos.x + Params.epsilon * Math.cos(theta);
            u.pos.y = v.pos.y + Params.epsilon * Math.sin(theta);
        }
        if(Double.isNaN(u.pos.x) || Double.isInfinite(u.pos.x) || Double.isNaN(u.pos.y) || Double.isInfinite(u.pos.y)){
            return;
        }
        // 检查点在障碍物之外
        if(!super.checkSample(u)){
            return;
        }
        // 连线在障碍物之外
        if(super.checkCollision(u,v)){
            if(closestNeighbours.size() < Params.maxNeighbours){
                this.addNode(u,v);
                rewireRand.add(u);
            }
            else {
                rewireRand.add(v);
            }
            rewireRandNode();
        }
        rewireFromRoot();
    }

    protected boolean checkSample(RRTNode n){
        for(Obstacle o:moving_obstacles){
            if(o.checkPosInObstacle(n.pos))
                return false;
        }
        return super.checkSample(n);
    }

    protected boolean checkCollision(RRTNode s, RRTNode e){
        for(Obstacle o:moving_obstacles){
            if(o.checkLineCrossObstacle(s.pos, e.pos) || o.checkPosInObstacle(s.pos) || o.checkPosInObstacle(e.pos))
                return false;
        }
        return super.checkCollision(s,e);
    }

    /**
     * 在随机点附近对rrtTree进行重连
     */
    private void rewireRandNode(){
        while (!rewireRand.isEmpty() && (System.currentTimeMillis() - timeKeeper < 0.5*Params.allowedTimeRewiring)){
            RRTNode Xr = rewireRand.poll();
            List<RRTNode> nearNodes = super.findClosestNeighbours(Xr);
            List<RRTNode> safeNeighbours = new LinkedList<>();
            for (RRTNode n:nearNodes){
                if(checkCollision(Xr, n)){
                    safeNeighbours.add(n);
                }
            }
            if(safeNeighbours.isEmpty()) continue;

            double cost_ = cost(Xr);
            for(RRTNode n:safeNeighbours){
                double oldCost = cost(n);
                double newCost = cost_ + getEuclideanDistance(n, Xr);
                if(oldCost > newCost){
                    rrt.getNode(n.parentID).children.remove(new Integer(n.nodeID));
                    n.parentID = Xr.nodeID;
                    n.costToRoot = newCost;
                    rrt.addChildID(Xr.nodeID, n.nodeID);
                    rewireRand.add(n);
                }

            }
        }
    }

    /**
     * 在根节点附近对rrtTree进行重连
     */
    private void rewireFromRoot(){
        if(rewireRoot.isEmpty()){
            rewireRoot.add(root);
        }

        while (!rewireRoot.isEmpty() && (System.currentTimeMillis() - timeKeeper) < Params.allowedTimeRewiring){
            RRTNode Xs = rewireRoot.poll();
            List<RRTNode> nearNodes = super.findClosestNeighbours(Xs);
            List<RRTNode> safeNeighbours = new LinkedList<>();
            for (RRTNode n:nearNodes){
                if(checkCollision(Xs, n)){
                    safeNeighbours.add(n);
                }
            }
            if(safeNeighbours.isEmpty()) continue;
            safeNeighbours.remove(rrt.getNode(Xs.parentID));

            double cost_ = cost(Xs);
            for(RRTNode n:safeNeighbours){
                double oldCost = cost(n);
                double newCost = cost_ + getEuclideanDistance(n, Xs);
                if(oldCost > newCost){
                    rrt.getNode(n.parentID).children.remove(new Integer(n.nodeID));
                    n.parentID = Xs.nodeID;
                    n.costToRoot = newCost;
                    rrt.addChildID(Xs.nodeID, n.nodeID);
                }
                // 如果在当前iter没有加入过rewireRoot
                if(!pushToRewireRoot.contains(n)){
                    rewireRoot.add(n);
                    pushToRewireRoot.add(n);
                }
            }
        }
    }

    /**
     * 利用检测到的动态障碍物阻塞节点
     * @param agent
     */
    public void blockNodes(Robot agent){
        for(CircleObstacle o:moving_obstacles){
            // 如果是在扫面范围内的障碍物
            if(getEuclideanDistance(o.getPos(), agent.getLocation())< Params.sensorRadius + o.getRadius()){
                // 阻塞在障碍物里面的节点
                for(RRTNode n:rrt.getRrtTree()){
                    if(o.checkPosInObstacle(n.pos)){
//                        System.out.println("block node"+ n.pos);
                        n.costToRoot = Double.POSITIVE_INFINITY;
                    }
                }
            }
        }
    }

    /**
     * 向树中添加节点
     * @param n
     * @param closest
     */
    private void addNode(RRTNode n, RRTNode closest){
        RRTNode parent = closest;
        double c_min = cost(closest) + getEuclideanDistance(n, parent);

        for(int i=0;i<closestNeighbours.size();++i){
            double c_new = cost(closestNeighbours.get(i)) + getEuclideanDistance(closestNeighbours.get(i).pos, n.pos);
            if(c_new < c_min && super.checkCollision(n, closestNeighbours.get(i))){
                c_min = c_new;
                parent = closestNeighbours.get(i);
                n.costToRoot = c_min;
            }
        }
        n.parentID = parent.nodeID;
        n.nodeID = rrt.getTreeSize();
        parent.children.add(n.nodeID);
        rrt.getRrtTree().add(n);

        if(getEuclideanDistance(n.pos, endPos) < Params.converge){
//            System.out.println("find path to goal");
            if(target == null || (target != null && target.costToRoot > n.costToRoot)){
                target = n;
            }
            goalFound = true;
        }
    }

    /**
     * 获取最近的一个节点
     * @param u
     * @return
     */
    private RRTNode getClosestNeighbour(RRTNode u){
        double min_dist = getEuclideanDistance(u.pos, rrt.getNode(0).pos);
        RRTNode near_node = rrt.getNode(0);
        for(int i=1;i<rrt.getTreeSize();++i){
            double dist = getEuclideanDistance(u.pos, rrt.getNode(i).pos);
            if(dist < min_dist){
                min_dist = dist;
                near_node = rrt.getNode(i);
            }
            if(dist < Params.rrtstarradius){
                // 加入邻居
                closestNeighbours.add(rrt.getNode(i));
            }
        }
        return near_node;
    }

    /**
     * 三种采样方式
     * @return
     */
    protected RRTNode sample(){
        double rand_num = Math.random();
        RRTNode n = null;
        // 在线上取点。。。
        if(rand_num > 1-Params.alpha && SMP.target != null){
            double x = rand(SMP.root.pos.x, SMP.target.pos.x);
            double y = rand(SMP.root.pos.y, SMP.target.pos.y);
            RRTNode new_node = new RRTNode();
            new_node.pos.x = x;
            new_node.pos.y = y;
            n = new_node;
        }
        else if(rand_num >= (1-Params.alpha)/Params.beta && goalFound){
            n = sample(cost(target));
        }
        else {
            n = super.sample();
        }

        return n;

    }

    /**
     * 找出路径代价，同时更新节点cost
     * @param node
     * @return
     */
    private double cost(RRTNode node){
        boolean badNode = false;
        double cost = 0;
        RRTNode curr = node;
        while (curr.parentID != -1){
            if(rrt.getNode(curr.parentID).costToRoot == Double.POSITIVE_INFINITY){
                curr.costToRoot = Double.POSITIVE_INFINITY;
                badNode = true;
                break;
            }
            cost += getEuclideanDistance(curr.pos, rrt.getNode(curr.parentID).pos);
            curr = rrt.getNode(curr.parentID);
        }
        if(badNode){
            return Double.POSITIVE_INFINITY;
        }
        else {
            node.costToRoot = cost;
            return cost;
        }
    }

    /**
     * 恢复初始状态
     */
    public void clearAll(){
        rewireRand = new LinkedList<>();
        rewireRoot = new LinkedList<>();
        closestNeighbours = new LinkedList<>();
        pushToRewireRoot = new LinkedList<>();
        currPath = new LinkedList<>();
        visited_set = new LinkedList<>();
        target = null;
        rrt.getRrtTree().clear();
        root = new RRTNode();
        root.parentID = -1;
        root.costToRoot = 0;
        root.nodeID = 0;
        root.pos.x = startPos.x;
        root.pos.y = startPos.y;
        rrt.getRrtTree().add(root);
        goalFound = false;
        sampledInGoalRegion = false;
    }
}

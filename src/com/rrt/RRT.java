package com.rrt;

import javax.vecmath.Vector2d;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

public class RRT {
    private List<RRTNode> rrtTree;
    private double getEuclideanDistance(Vector2d source, Vector2d dest){
        Vector2d temp = new Vector2d();
        temp.sub(dest,source);
        return temp.length();
    }

    public RRT(){
        rrtTree = new ArrayList<>();
        RRTNode rrtNode = new RRTNode();
        rrtNode.pos.x = 0;
        rrtNode.pos.y = 0;
        rrtNode.costToRoot = 0;
        rrtNode.nodeID = 0;
        rrtNode.parentID = -1;
        rrtTree.add(rrtNode);
    }

    public RRT(double startX, double startY){
        this();
        rrtTree.get(0).pos.x = startX;
        rrtTree.get(0).pos.y = startY;
    }

    public List<RRTNode> getRrtTree() {
        return rrtTree;
    }

    public void setRrtTree(List<RRTNode> rrtTree) {
        this.rrtTree = rrtTree;
    }

    public int getTreeSize(){
        return rrtTree.size();
    }

    public RRTNode getNode(int nodeID){
        if(nodeID < 0)
            return null;
        return rrtTree.get(nodeID);
    }

    public double getPosX(int nodeID){
        return rrtTree.get(nodeID).pos.x;
    }

    public double getPosY(int nodeID){
        return rrtTree.get(nodeID).pos.y;
    }

    public void setPosX(int nodeID, double X){
        rrtTree.get(nodeID).pos.x = X;
    }

    public void setPosY(int nodeID, double Y){
        rrtTree.get(nodeID).pos.y = Y;
    }

    public RRTNode getParent(int nodeID){
        return rrtTree.get(rrtTree.get(nodeID).parentID);
    }

    public void setParent(int nodeID, int parentID){
        rrtTree.get(nodeID).parentID = parentID;
    }

    public void addChildID(int nodeID, int chhildID){
        rrtTree.get(nodeID).children.add(chhildID);
    }

    public List<Integer> getChildren(int nodeID){
        return rrtTree.get(nodeID).children;
    }

    public void setCostToRoot(int nodeID, double cost){
        getNode(nodeID).costToRoot = cost;
    }

    public void addNode(RRTNode node, int parentID){
        node.parentID = parentID;
        node.nodeID = rrtTree.size();
//        node.costToRoot = getNode(parentID).costToRoot + getEuclideanDistance(node.pos, getNode(parentID).pos);
        rrtTree.add(node);
        addChildID(parentID, node.nodeID);
    }

    public int getNearestNodeID(Vector2d pos){
        double distance = 999999;
        int id = 0;
        for(RRTNode node: rrtTree){
            double tempDistance = getEuclideanDistance(pos, node.pos);
            if(tempDistance < distance){
                distance = tempDistance;
                id = node.nodeID;
            }
        }
        return id;
    }

    public List<Integer> getRootToEndPath(int endNodeID){
        int nodeID = endNodeID;
        Queue<Integer> path = new ArrayDeque<>();
        while (nodeID != 0){
            path.add(nodeID);
            nodeID = rrtTree.get(nodeID).parentID;
        }
        path.add(0);
        return new ArrayList<>(path);
    }

    public List<Integer> getNeighbours(Vector2d pos, double neighbourhood){
        List<Integer> neighbours = new ArrayList<>();
        for(RRTNode node: rrtTree){
            if(getEuclideanDistance(pos, node.pos) <= neighbourhood){
                neighbours.add(node.nodeID);
            }
        }
        return neighbours;
    }

}


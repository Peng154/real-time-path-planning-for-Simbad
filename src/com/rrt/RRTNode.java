package com.rrt;

import javax.vecmath.Vector2d;
import java.util.ArrayList;
import java.util.List;

public class RRTNode {
    public int nodeID; // 节点编号
    public int parentID; // 父亲节点编号
    public List<Integer> children; // 子节点编号
    public Vector2d pos; // 二维空间位置
    public double costToRoot; // 从root到本节点的cost

    public RRTNode(){
        children = new ArrayList<>();
        pos = new Vector2d();
    }
}

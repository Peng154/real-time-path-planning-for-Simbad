package com.rt_rrt;


import com.base.Obstacle;
import com.rrt.RRTNode;

import javax.vecmath.Vector2d;
import java.util.List;

public class InformedRRTStar extends RRTStar{
    
    public boolean usingInformedRRTstar = false;
    /**
     * 路径规划器的基本元素是：已知障碍物、起始点、终点、规划边界
     *
     * @param obstacles  障碍物的点集列表，每个障碍物的点按照顺时针方向排列,左上角为第一个点（目前的障碍物都是矩形并且不旋转）
     * @param startPos
     * @param endPos
     * @param boundaries
     */
    public InformedRRTStar(List<Obstacle> obstacles, Vector2d startPos, Vector2d endPos, List<Double> boundaries) {
        super(obstacles, startPos, endPos, boundaries);
    }

    protected RRTNode sample(double c_max){
        double c_min = getEuclideanDistance(endPos, startPos);


        if (Math.abs(c_max - c_min) < 100 && usingInformedRRTstar) //Putting a dummy value for now - Robot might not move for some configurations with this value
        SMP.moveNow = true; //TODO: The flag will be associated with time. Should turn on when the spcified time lapses

        Vector2d x_centre = new Vector2d();
        x_centre.x = (startPos.x + endPos.x)/2;
        x_centre.y = (startPos.y + endPos.y)/2;
        Vector2d dir = new Vector2d();
        dir.sub(endPos, startPos);
        dir.normalize();
        double angle = Math.atan2(dir.y, dir.x);
        double r1 = c_max / 2;
        double r2 = Math.sqrt(Math.pow(c_max, 2) - Math.pow(c_min, 2)) / 2;

        double x = rand(-1, 1);
        double y = rand(-1, 1);

        double x2 = x * r1 * Math.cos(angle) - y * r2 * Math.sin(angle);
        double y2 = x * r1 * Math.sin(angle) + y * r2 * Math.cos(angle);

        Vector2d rot_sample = new Vector2d(x2,y2), rot_trans_sample = new Vector2d();
        rot_trans_sample.add(rot_sample, x_centre);

        RRTNode n = new RRTNode();
        n.pos.x = rot_trans_sample.x;
        n.pos.y = rot_trans_sample.y;

        return n;
    }
}

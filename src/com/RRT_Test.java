package com;

import com.base.Obstacle;
import com.base.RectObstacle;
import com.rrt.RRTNode;
import com.rrt.RRTPlanner;
import com.rrt_star.RRTStarPlanner;
import com.base.Visualizer;

import javax.vecmath.Vector2d;
import java.util.ArrayList;
import java.util.List;

public class RRT_Test {

    protected double getEuclideanDistance(Vector2d source, Vector2d dest){
        Vector2d temp = new Vector2d();
        temp.sub(dest,source);
        return temp.length();
    }

    protected double rand(double a, double b){
        assert b>a;
        double r = Math.random();
        return a+r*(b-a);
    }

    protected RRTNode sample(double c_max){
        Vector2d startPos = new Vector2d(0,0);
        Vector2d endPos = new Vector2d(2,0);

        double c_min = getEuclideanDistance(endPos, startPos);


//        if (Math.abs(c_max - c_min) < 100 && usingInformedRRTstar) //Putting a dummy value for now - Robot might not move for some configurations with this value
//            SMP.moveNow = true; //TODO: The flag will be associated with time. Should turn on when the spcified time lapses

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

        double x2 = x * r1 * Math.cos(angle) + y * r2 * Math.sin(angle);
        double y2 = - x * r1 * Math.sin(angle) + y * r2 * Math.cos(angle);

        Vector2d rot_sample = new Vector2d(x2,y2), rot_trans_sample = new Vector2d();
        rot_trans_sample.add(rot_sample, x_centre);

        RRTNode n = new RRTNode();
        n.pos.x = rot_trans_sample.x;
        n.pos.y = rot_trans_sample.y;

        System.out.println(rot_trans_sample.x);
        System.out.println(rot_trans_sample.y);

        return n;
    }

    public static void main(String[] args){
        List<Obstacle> obstacles = new ArrayList<>();
        List<Vector2d> pos = new ArrayList<>();
        pos.add(new Vector2d(-50,50));
        pos.add(new Vector2d(50,50));
        pos.add(new Vector2d(50,-50));
        pos.add(new Vector2d(-50,-50));
        Obstacle obs = new RectObstacle(0, pos);
        obstacles.add(obs);
        Vector2d s = new Vector2d(-60,-60);
        Vector2d e = new Vector2d(60,60);
        List<Double> b = new ArrayList<>();
        b.add(-90.0);
        b.add(90.0);
        b.add(-90.0);
        b.add(90.0);

        RRTPlanner p1 = new RRTPlanner(obstacles,s,e,b, 10, 4);
        p1.plan();

        RRTStarPlanner p2 = new RRTStarPlanner(obstacles,s,e,b, 10, 6);
        p2.plan();

        Visualizer v1 = new Visualizer(p1, 5);
        v1.show();

        Visualizer v2 = new Visualizer(p2, 5);
        v2.show();

        RRTNode n = new RRT_Test().sample(3);
    }
}

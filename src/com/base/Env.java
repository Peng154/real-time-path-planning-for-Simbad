package com.base;

import com.rt_rrt.Params;
import com.rt_rrt.SMP;
import simbad.sim.Agent;
import simbad.sim.Box;
import simbad.sim.EnvironmentDescription;
import simbad.sim.Wall;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import com.rt_rrt.Robot;

import java.util.ArrayList;
import java.util.List;

/** Describe the environement */
public class Env extends EnvironmentDescription {
    public Env() {
        light1IsOn = true;
        light2IsOn = false;
        // --------------------------------------------------------------------墙
        Wall w1 = new Wall(new Vector3d(9, 0, 0), 19, 1, this);
        w1.rotate90(1);
        add(w1);
        Wall w2 = new Wall(new Vector3d(-9, 0, 0), 19, 2, this);
        w2.rotate90(1);
        add(w2);
        Wall w3 = new Wall(new Vector3d(0, 0, 9), 19, 1, this);
        add(w3);
        Wall w4 = new Wall(new Vector3d(0, 0, -9), 19, 2, this);
        add(w4);

        // -------------------------------------------------------------------静态障碍物
        List<Obstacle> obstacles = new ArrayList<>();
        Box b1 = new Box(new Vector3d(0, 0, 6), new Vector3f(1f, 1f, 7f), this);
        add(b1);
        Obstacle obs1 = new RectObstacle(0.35, new Vector2d(0, 4), 1, 7);
        obstacles.add(obs1);

        Box b2 = new Box(new Vector3d(0, 0, -5.3), new Vector3f(1f, 1f, 6f), this);
        add(b2);
        Obstacle obs2 = new RectObstacle(0.35, new Vector2d(0, -5.3), 1, 6);
        obstacles.add(obs2);

        Box b3 = new Box(new Vector3d(5.5, 0, -3), new Vector3f(5f, 1f, 1f), this);
        add(b3);
        Obstacle obs3 = new RectObstacle(0.35, new Vector2d(5.5, -3), 5, 1);
        obstacles.add(obs3);

        Box b4 = new Box(new Vector3d(-3, 0, 0), new Vector3f(1f, 1f, 7f), this);
        add(b4);
        Obstacle obs4 = new RectObstacle(0.35, new Vector2d(-3, 0), 1, 7);
        obstacles.add(obs4);


//        Box test = new Box(new Vector3d(0, 0, -1.93), new Vector3f(10f, 1f, 0.1f), this);
//        add(test);

        // -----------------------------------------------------------------动态障碍物
        List<CircleObstacle> moving_obs = new ArrayList<>();
        CircleObstacle mo1 = new CircleObstacle(0.4, 0.3, new Vector2d(0,0));
        moving_obs.add(mo1);
        DynamicObstacle do1 = new DynamicObstacle(new Vector3d(0, 0, 2), "mo1", mo1, 3.7);
        add(do1);

        CircleObstacle mo2 = new CircleObstacle(0.4, 0.3, new Vector2d(0,0));
        moving_obs.add(mo2);
//        DynamicObstacle do2 = new DynamicObstacle(new Vector3d(4, 0, -4.7), "mo2", mo2, 3.3);
        DynamicObstacle do2 = new DynamicObstacle(new Vector3d(4, 0, -4.7), "mo2", mo2);
        add(do2);

        CircleObstacle mo3 = new CircleObstacle(0.4, 0.3, new Vector2d(0,0));
        moving_obs.add(mo3);
//        DynamicObstacle do3 = new DynamicObstacle(new Vector3d(-2, 0, -4.7), "mo3", mo3, 2.5);
        DynamicObstacle do3 = new DynamicObstacle(new Vector3d(-2, 0, -3), "mo3", mo3, 4);
        add(do3);



        Vector2d s = new Vector2d(-5.9,-5.9);
        Vector2d e = new Vector2d(6,-7.5);
        List<Double> b = new ArrayList<>();
        b.add(-8.5);
        b.add(8.5);
        b.add(-8.5);
        b.add(8.5);
//
        add(new Robot(new Vector3d(-6, 0, -6), "robot 1", obstacles, s, e, b, moving_obs));


    }
}

/**
 * 运动的障碍物
 */
class DynamicObstacle extends Agent {

    private CircleObstacle obstacle;
    private boolean circle = true;
    private Vector2d startPos = null;
    private Vector2d endPos = null;
    private Vector2d target = null;

    public DynamicObstacle(Vector3d arg0, String arg1, CircleObstacle obstacle) {
        super(arg0, arg1);
        this.obstacle = obstacle;
    }

    protected double getEuclideanDistance(Vector2d source, Vector2d dest){
        Vector2d temp = new Vector2d();
        temp.sub(dest,source);
        return temp.length();
    }

    public DynamicObstacle(Vector3d arg0, String arg1, CircleObstacle obstacle, double length) {
        super(arg0, arg1);
        this.obstacle = obstacle;
        circle = false;
        // 设置横向移动终点
        startPos = new Vector2d();
        startPos.x = arg0.x;
        startPos.y = arg0.z;
        endPos = new Vector2d();
        endPos.x = startPos.x;
        endPos.y = startPos.y - length;
        target = endPos;
    }

    /**
     * 获取二维坐标
     * @return
     */
    public Vector2d getLocation(){
        Vector2d pos = new Vector2d();
        Point3d pos_ = new Point3d();
        getCoords(pos_);
        pos.x = pos_.x;
        pos.y = pos_.z;
        return pos;
    }

    @Override
    protected void initBehavior() {
        if(circle){
            setRotationalVelocity(Params.dyanamicRotationSpeed);
            setTranslationalVelocity(2 * 1.5* Params.dyanamicRotationSpeed * getRadius());
        }
    }

    @Override
    protected void performBehavior() {
        // 更新动态机器人位置
        obstacle.setPos(getLocation());

        if(target == null){
            return;
        }

        Vector2d pos = getLocation();
        // 到达目标点，停下来
        if(getEuclideanDistance(target, getLocation()) < 0.05){
//                System.out.println("get target");
            setTranslationalVelocity(0);
            setRotationalVelocity(0);
        }else{
            // 获取机器人角度
            double[] mat = new double[16];
            Transform3D t = new Transform3D();
            this.getRotationTransform(t);
            t.get(mat);
            double theta = -Math.atan2(mat[2],mat[10]);
            // 目标点角度
            double alpha = Math.atan2(target.y-pos.y, target.x-pos.x);
            double error = theta - alpha;

//                System.out.println("theta: "+theta+", alpha:"+alpha);
            if(Math.abs(error) < 0.08){
//                    System.out.println("right direction");
                // 角度对了，不旋转
                setRotationalVelocity(0);
                setTranslationalVelocity(Params.lineDynamicRobotSpeed);
            }
            else{
                // 角度不对，只旋转
                if(error > 0){
                    setRotationalVelocity(3);
                }
                else {
                    setRotationalVelocity(-3);
                }
            }
        }

        if(getEuclideanDistance(pos, endPos) < 0.05){
            target = startPos;
        }
        else if(getEuclideanDistance(pos, startPos) < 0.05){
            target = endPos;
        }

    }
}

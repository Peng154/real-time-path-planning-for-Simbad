package com.rt_rrt;

import com.base.CircleObstacle;
import com.base.Obstacle;
import com.rrt.RRTNode;
import simbad.sim.Agent;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import java.util.Deque;
import java.util.List;

/** Describe the robot */
public class Robot extends Agent {

    RTRRTStar rtrrtStar;
    private boolean planner = true;
    private Deque<RRTNode> path;

    public Robot(Vector3d position, String name, List<Obstacle> obstacles, Vector2d startPos, Vector2d endPos, List<Double> boundaries, List<CircleObstacle> m_obs) {
        super(position, name);
        rtrrtStar = new RTRRTStar(obstacles, startPos, endPos, boundaries, m_obs);
    }

    /** This method is called by the simulator engine on reset. */
    public void initBehavior() {
        // nothing particular in this case
        rtrrtStar.clearAll();
        planner = true;
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

    protected double getEuclideanDistance(Vector2d source, Vector2d dest){
        Vector2d temp = new Vector2d();
        temp.sub(dest,source);
        return temp.length();
    }

    /** This method is call cyclically (20 times per second)  by the simulator engine. */
    public void performBehavior() {


        if(getEuclideanDistance(getLocation(), rtrrtStar.endPos) < Params.converge){
            planner = false;
            setTranslationalVelocity(0);
            setRotationalVelocity(0);
        }

        if(planner){

            // 根据移动机器人阻塞节点
            rtrrtStar.blockNodes(this);

            Vector2d target = SMP.root.pos;
            Vector2d pos = getLocation();
//            if(getCounter() % 40 == 0){
//                System.out.println(target);
//                System.out.println(pos);
//            }
            // 到达目标点，停下来
            if(getEuclideanDistance(target, getLocation()) < Params.nearError){
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
                    setTranslationalVelocity(Params.robotSpeed);
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
            rtrrtStar.nextIter(this);
        }


        if(planner && SMP.target!=null){
            path = rtrrtStar.currPath;
            rtrrtStar.currPath.clear();
        }
    }
}
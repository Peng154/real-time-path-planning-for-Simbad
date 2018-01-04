package com.base;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

public class RectObstacle extends Obstacle {
    private List<Vector2d> points;
    private List<Vector2d> expanded_points;

    /**
     *
     * @param safe_radius 预留安全半径
     * @param points 顺时针方向的点，左上角在第一个
     */
    public RectObstacle(double safe_radius, List<Vector2d> points) {
        super(safe_radius);
        assert points.size() == 4; // 一定要有四个点
        this.points = points;
        // 扩宽障碍物
        expanded_points = new ArrayList<>();
        expandArea();
    }

    public RectObstacle(double safe_radius, Vector2d center, double xl, double yl){
        super(safe_radius);
        points = new ArrayList<>();
        points.add(new Vector2d(center.x - 0.5* xl,center.y + 0.5 * yl));
        points.add(new Vector2d(center.x + 0.5* xl,center.y + 0.5 * yl));
        points.add(new Vector2d(center.x + 0.5* xl,center.y - 0.5 * yl));
        points.add(new Vector2d(center.x - 0.5* xl,center.y - 0.5 * yl));
        // 扩宽障碍物
        expanded_points = new ArrayList<>();
        expandArea();
    }

    private void expandArea(){
        for(int i=0;i<points.size();++i){
            Vector2d temp = new Vector2d(points.get(i));
            switch (i){
                case 0:
                    temp.x -= safe_radius;
                    temp.y += safe_radius;
                    break;
                case 1:
                    temp.x += safe_radius;
                    temp.y += safe_radius;
                    break;
                case 2:
                    temp.x += safe_radius;
                    temp.y -= safe_radius;
                    break;
                case 3:
                    temp.x -= safe_radius;
                    temp.y -= safe_radius;
                    break;
            }
            expanded_points.add(temp);
        }
    }

    /**
     * 在预留安全距离的情况下确认线段是否穿过障碍物
     * @param s
     * @param e
     * @return
     */
    @Override
    public boolean checkLineCrossObstacle(Vector2d s, Vector2d e) {
        return checkLineCrossRect(s,e,expanded_points);
    }

    /**
     * 在预留安全距离的情况下确认点是否在障碍物内
     * @param pos
     * @return
     */
    @Override
    public boolean checkPosInObstacle(Vector2d pos) {
        return checkPosInRect(pos, expanded_points);
    }

    public List<Vector2d> getPoints() {
        return points;
    }

    private boolean checkPosInRect(Vector2d pos, List<Vector2d> rect){
        double AB, AD, AMAB, AMAD;
        AB = Math.pow((rect.get(1).x - rect.get(0).x),2) + Math.pow((rect.get(1).y - rect.get(0).y),2);
        AD = Math.pow((rect.get(3).x - rect.get(0).x),2) + Math.pow((rect.get(3).y - rect.get(0).y),2);
        AMAB = (pos.x - rect.get(0).x)*(rect.get(1).x - rect.get(0).x) +
                (pos.y - rect.get(0).y)*(rect.get(1).y - rect.get(0).y);
        AMAD = (pos.x - rect.get(0).x)*(rect.get(3).x - rect.get(0).x) +
                (pos.y - rect.get(0).y)*(rect.get(3).y - rect.get(0).y);

        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return true;
        }
        return false;
    }

    private Vector3d MatrixDotVector(Matrix3d m, Vector3d v){
        Vector3d r0 = new Vector3d();
        Vector3d r1 = new Vector3d();
        Vector3d r2 = new Vector3d();
        m.getColumn(0, r0);
        m.getColumn(1, r1);
        m.getColumn(2, r2);
        Vector3d result = new Vector3d();
        result.x = r0.dot(v);
        result.y = r1.dot(v);
        result.z = r2.dot(v);
        return result;
    }

    /**
     * 计算line1和line2两条线段是否相交
     * 先整体平移两条线段至line1的起点到原点
     * 然后整体旋line1和x轴相交
     * 最后只需判断平移和旋转之后的line2是否和x轴有交点并且交点的x值是否在line1线段内
     * @param s1 line1的起点
     * @param e1 line1的终点
     * @param s2 line2的起点
     * @param e2 line2的终点
     * @return
     */
    private boolean checkLineCross(Vector2d s1, Vector2d e1, Vector2d s2, Vector2d e2){
        // 整体平移至其中一条线段的起点到原点
        Vector2d sm1 = new Vector2d(0,0);
        Vector2d em1 = new Vector2d();
        em1.sub(e1, s1);
        Vector2d sm2 = new Vector2d();
        sm2.sub(s2, s1);
        Vector2d em2 = new Vector2d();
        em2.sub(e2, s1);

        // 计算夹角
        double theta = Math.atan2(em1.y, em1.x);
        Matrix3d M = new Matrix3d(Math.cos(theta), -Math.sin(theta), 0, Math.sin(theta), Math.cos(theta),
                0, 0, 0, 1);
        // 旋转
        Vector3d before = new Vector3d(em1.x, em1.y, 1);
        Vector3d after = MatrixDotVector(M, before);
        em1.x = after.x;
        em1.y = after.y;

        before = new Vector3d(sm2.x, sm2.y, 1);
        after = MatrixDotVector(M, before);
        sm2.x = after.x;
        sm2.y = after.y;

        before = new Vector3d(em2.x, em2.y, 1);
        after = MatrixDotVector(M, before);
        em2.x = after.x;
        em2.y = after.y;

//        System.out.println("sm1:" +sm1.x+" "+sm1.y);
//        System.out.println("em1:" +em1.x+" "+em1.y);
//        System.out.println("sm2:" +sm2.x+" "+sm2.y);
//        System.out.println("em2:" +em2.x+" "+em2.y);


        // 如果line2和x轴存在交点
        if(sm2.y * em2.y < 0){
            // 计算交点的x值
            Vector2d crossPos = new Vector2d();
            double beta = Math.atan2(em2.y-sm2.y, em2.x-sm2.x);
            double x = em2.x - (em2.y / Math.tan(beta));
            if(x >= sm1.x && x <= em1.x){
                return true;
            }
        }

        return false;
    }

    /**
     * 直线是否和矩形有交点
     * @param s
     * @param e
     * @param rect
     * @return
     */
    private boolean checkLineCrossRect(Vector2d s, Vector2d e, List<Vector2d> rect){
        for(int i=1;i<rect.size();++i){
            // 如果两条线段相交，那么线段就不在障碍物外面
            if(checkLineCross(s,e, rect.get(i), rect.get(i-1))){
                return true;
            }
        }
        return false;
    }

}

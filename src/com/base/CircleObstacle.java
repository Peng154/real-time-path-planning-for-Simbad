package com.base;

import javax.vecmath.Vector2d;

public class CircleObstacle extends Obstacle{
    private Vector2d pos;
    private double radius;
    private double expanded_radius;
    public CircleObstacle(double safe_radius, double radius, Vector2d pos) {
        super(safe_radius);
        this.radius = radius;
        this.pos = pos;
        this.expanded_radius = radius + safe_radius;
    }

    /**
     * 直线是否穿过圆形障碍物
     * @param s
     * @param e
     * @return
     */
    @Override
    public boolean checkLineCrossObstacle(Vector2d s, Vector2d e) {
        double x1 = s.x;
        double x2 = e.x;
        double y1 = s.y;
        double y2 = e.y;

        double xo = pos.x;
        double yo = pos.y;
        double lambda = Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2);
        double t = (Math.pow(x1, 2) + x2 * xo - x1*(x2 + xo) - (yo - y1)*(y1 - y2)) / lambda;
        double shortest_dist;
        if (t >= 0 && t <= 1) // If the perpendicular distance lies on the line 'segment' connecting point_1 and point_2
            shortest_dist = Math.sqrt(Math.pow((x2 * (y1 - yo) + x1 * (yo - y2) + xo * (y2 - y1)), 2) / lambda);
	else  // If not then only check for the end-points of the segment for the collision
        {
            double d1 = Math.sqrt(Math.pow((x1 - xo), 2) + Math.pow((y1 - yo), 2));
            double d2 = Math.sqrt(Math.pow((x2 - xo), 2) + Math.pow((y2 - yo), 2));
            shortest_dist = Math.min(d1, d2);
        }

        if (shortest_dist < expanded_radius) 	return true;
        return false;
    }

    @Override
    public boolean checkPosInObstacle(Vector2d pos) {
        if(getEuclideanDistance(pos, this.pos) > expanded_radius){
            return false;
        }
        else {
            return true;
        }
    }

    public Vector2d getPos() {
        return pos;
    }

    public void setPos(Vector2d pos) {
        this.pos = pos;
    }

    public double getRadius() {
        return radius;
    }
}

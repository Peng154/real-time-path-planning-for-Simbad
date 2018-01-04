package com.base;


import com.base.Obstacle;
import com.base.Planner;
import com.base.RectObstacle;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

import javax.swing.*;
import javax.vecmath.Vector2d;
import java.awt.*;
import java.util.List;


public class Visualizer {

    private Planner planner;
    private int factor;

    public Visualizer(Planner planner, int reszieFactor){
        this.planner = planner;
        this.factor = reszieFactor;
    }

//    public static void main(String[] args){
//        new  Visualizer(null).show();
//    }

    public void show() {
        JFrame frame = new JFrame();

        frame.getContentPane().add(new Painter());
        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    @SuppressWarnings("serial")
    private class Painter extends JPanel {
        private int error;

        public Painter() {

            setBackground(Color.white);
            this.error = (int)(planner.getBoundaries().get(1) * factor);
            setPreferredSize(new Dimension(error * 2, error * 2));
            repaint();
        }

        public void repaint() {
            super.repaint();
        }

        private int transformX(double x){
            return (int)x * factor + error;
        }

        private int transformY(double y){
            return -((int)y * factor - error);
        }

        private void drawStartEndPoint(Graphics page){
            int sx = transformX(planner.getStartPos().x);
            int sy = transformY(planner.getStartPos().y);
            int ex = transformX(planner.getEndPos().x);
            int ey = transformY(planner.getEndPos().y);

            page.drawRoundRect(sx,sy,10,10,10,10);
            page.drawRoundRect(ex,ey,10,10,10,10);
        }

        private void drawObstacles(Graphics page){
            for(Obstacle o:planner.getObstacles()){
                if(o instanceof RectObstacle){
                    RectObstacle ro = (RectObstacle)o;
                    List<Vector2d> pos = ro.getPoints();
//                System.out.println("ooooo");
                    Vector2d rec = new Vector2d();
                    rec.sub(pos.get(2), pos.get(0));
                    int width = (int)Math.abs(rec.x) * factor;
                    int height = (int)Math.abs(rec.y) * factor;
                    int x = transformX(pos.get(0).x);
                    int y = transformY(pos.get(0).y);
                    page.drawRect(x, y, width, height);
                }
                else {
                    throw new  NotImplementedException();
                }

            }
        }

        private void drawPath(Graphics page){
            List<Vector2d> path = planner.getBestPath();

            if(path == null){
                System.out.println("未找到合理路径。。。。");
            }

            int lastX = transformX(path.get(0).x);
            int lastY = transformY(path.get(0).y);

            for(int i=1;i<path.size();++i){
                int currX = transformX(path.get(i).x);
                int currY = transformY(path.get(i).y);

//                System.out.println("pos:"+currX+", "+currY);
                page.drawLine(lastX, lastY, currX, currY);
                lastX = currX;
                lastY = currY;
            }
        }

        public void paint(Graphics page) {
            drawStartEndPoint(page);
            drawObstacles(page);
            drawPath(page);
        }
    }
}

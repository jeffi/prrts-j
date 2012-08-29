package edu.unc.robotics.prrts.example.geom;

import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;

/**
 * Circle
 *
 * @author jeffi
 */
public class Circle implements Obstacle {
    private double _x;
    private double _y;
    private double _r;
    private Ellipse2D.Double _shape;

    public Circle(double x, double y, double r) {
        _shape = new Ellipse2D.Double(x - r, y - r, r*2, r*2);
        _x = x;
        _y = y;
        _r = r;
    }

    @Override
    public Shape shape() {
        return _shape;
    }

    @Override
    public double distToPoint(double x, double y) {
        double dx = x - _x;
        double dy = y - _y;
        return Math.sqrt(dx * dx + dy * dy) - _r;
    }

    @Override
    public double distToSeg(double x1, double y1, double x2, double y2) {
        return Line2D.ptSegDist(x1, y1, x2, y2, _x, _y) - _r;
    }
}

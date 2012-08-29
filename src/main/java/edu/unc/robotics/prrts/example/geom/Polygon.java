package edu.unc.robotics.prrts.example.geom;

import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.util.Arrays;

/**
 * Polygon
 *
 * @author jeffi
 */
public class Polygon implements Obstacle {
    private Path2D.Double _shape;
    private double[] _coords;

    public Polygon(double ... coords) {
        _shape = new Path2D.Double();
        _shape.moveTo(coords[0], coords[1]);
        for (int i=2 ; i<coords.length ; i+=2) {
            _shape.lineTo(coords[i], coords[i+1]);
        }
        _shape.closePath();
        _coords = Arrays.copyOf(coords, coords.length);
    }

    @Override
    public Shape shape() {
        return _shape;
    }

    @Override
    public double distToPoint(double x, double y) {
        int n = _coords.length;

        // first calculate the distance from all walls, and choose the closest
        double dist = Double.MAX_VALUE;
        for (int i=0, j=n-2 ; i<n ; j=i, i+=2) {
            double d = Line2D.ptSegDistSq(_coords[j], _coords[j + 1], _coords[i], _coords[i + 1], x, y);
            if (d < dist) {
                dist = d;
            }
        }
        // Save the sqrt to the end
        dist = Math.sqrt(dist);

        // Next find out if the point is inside the shape or not.
        // if it is inside, return a negative distance to indicate
        // the point is in the interior
        if (_shape.contains(x, y)) {
            dist = -dist;
        }

        return dist;
    }

    @Override
    public double distToSeg(double x1, double y1, double x2, double y2) {
        final double[] c = _coords;
        final int n = c.length;

        for (int i=0, j=n-2 ; i<n ; j=i, i+=2) {
            if (Line2D.linesIntersect(
                x1, y1, x2, y2,
                c[j], c[j+1],
                c[i], c[i+1]))
            {
                return -1;
            }
        }

        if (_shape.contains(x1, y1) || _shape.contains(x2, y2)) {
            return -1;
        }

        double dist = Double.MAX_VALUE;
        for (int i=0, j=n-2 ; i<n ; j=i, i+=2) {
            dist = Math.min(
                Math.min(
                    Line2D.ptSegDistSq(c[j], c[j+1], c[i], c[i+1], x1, y1),
                    Line2D.ptSegDistSq(c[j], c[j+1], c[i], c[i+1], x2, y2)),
                Math.min(
                    Line2D.ptSegDistSq(x1, y1, x2, y2, c[i], c[i+1]),
                    dist));
        }

        return Math.sqrt(dist);
    }
}

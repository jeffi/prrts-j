package edu.unc.robotics.prrts.example.arena;

import edu.unc.robotics.prrts.RobotModel;
import edu.unc.robotics.prrts.example.geom.Circle;
import edu.unc.robotics.prrts.example.geom.Obstacle;
import edu.unc.robotics.prrts.example.geom.Polygon;
import edu.unc.robotics.prrts.kdtree.KDModel;

import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.logging.Logger;

/**
 * HolonomicArena2D
 *
 * @author jeffi
 */
public class HolonomicArena implements RobotModel, KDModel {
    private static final Logger _log = Logger.getLogger(HolonomicArena.class.getName());

    private static final double DISCRETIZATION = 0.25;

    public static final double ROBOT_RADIUS = .4;
    private static final double ROBOT_RADIUS_X2POW2 = ROBOT_RADIUS * ROBOT_RADIUS * 4;
    static final double GOAL_RADIUS = 0.4;
    private static final boolean GOAL_BIASED = false;

    double _eta = 1.0;

    double[] _goal = { 2.7, 9.0, 1, 1, 1, 9, 9, 1 };
    Shape _goalShape;
    int _dimensions = 8;
    double[] _min = { 0, 0, 0, 0, 0, 0, 0, 0 };
    double[] _max = { 10, 10, 10, 10, 10, 10, 10, 10 };

//    Obstacle[] _obstacles = new Obstacle[] {
//        new Circle(4.5, 6.5, 1.5), // new Ellipse2D.Double(3,4, 3,3),
////        new Rectangle2D.Double(8, 1, 1, 1),
////        new Rectangle2D.Double(3, 1.5, 5, .5),
//        new Circle(2.75, 1.75, 0.75), //  new Ellipse2D.Double(2, 1, 1.5, 1.5),
//        new Circle(7, 3, 1), // new Ellipse2D.Double(6, 2, 2, 2),
//        new Circle(8, 6, 1), // new Ellipse2D.Double(7, 5, 2, 2),
////        new Rectangle2D.Double(1, 4.5, 4, 1)
//        new Circle(2, 4, 1), //  new Ellipse2D.Double(1, 3, 2, 2),
//        new Circle(4.5, 3.5, .5), // new Ellipse2D.Double(4, 8, 1, 1),
//    };

    Obstacle[] _obstacles = new Obstacle[] {
        new Circle(2.0, 2.5, 1.1), // dinner table
        new Polygon(0.5, 4.5,   2.1, 4.5,  2.1, 8.0,  0.5, 8.0), // couch
        new Circle(1.5, 9.0, 1.5/2), // end table
        new Polygon(3.9, 8.5, 4.2, 9.9,  6.2, 9.5,  5.9, 8.1), // blue chair
        new Polygon(4.1, 5.4,  5.5, 5.4,  5.5, 7.0,  4.1, 7.0), // coffee table
        new Polygon(4.5, 2.5,  7.1, 2.5,  7.1, 3.5,  4.5, 3.5), // island
//        new Polygon(3.5, 3.8,  4.0, 4.3,  3.5, 4.8,  3.0, 4.4), // chair
        new Circle(3.6, 4.3, 0.35), // chair v2
        new Polygon(9.2, 2.5,  10.0, 2.5,  10.0, 3.5,  9.2, 3.5), // fridge
        new Circle(6.6, 4.1,  0.5), // toy
        new Polygon(9.2, 4.5, 10.0, 4.5, 10.0, 9.0,  9.2, 9.0), // wall
        new Circle(2.4, 7.0, 0.3), // ottoman
    };

    private Rectangle2D.Double _bounds = new Rectangle2D.Double(0, 0, 10, 10);

    public HolonomicArena(int robots) {
        if (robots < 1 || robots > 4) {
            throw new IllegalArgumentException();
        }
        _dimensions = robots*2;

        Area goalShape = new Area();
        for (int i=0 ; i<robots ; ++i) {
            goalShape.add(new Area(new Ellipse2D.Double(
                _goal[i*2] - GOAL_RADIUS, _goal[i*2+1] - GOAL_RADIUS,
                GOAL_RADIUS, GOAL_RADIUS)));
        }
        _goalShape = goalShape;
    }

    @Override
    public int dimensions() {
        return _dimensions;
    }

    @Override
    public void getBounds(double[] min, double[] max) {
        System.arraycopy(_min, 0, min, 0, _dimensions);
        System.arraycopy(_max, 0, max, 0, _dimensions);
    }

    //    @Override
//    public void randomize(Random rand, double[] c) {
//        for (int i=0 ; i<_dimensions ; ++i) {
//            c[i] = rand.nextDouble() * (_max[i] - _min[i]) + _min[i];
//        }
//    }

    @Override
    public double dist(double[] a, double[] b) {
        double dist = 0;
        for (int i=0 ; i<_dimensions ; i+=2) {
            double dx = a[i] - b[i];
            double dy = a[i+1] - b[i+1];
//            dist = Math.max(dist, Math.sqrt(dx*dx + dy*dy));
//            dist += Math.sqrt(dx*dx + dy*dy);
            dist += dx*dx + dy*dy;
        }
//        return dist;
        return Math.sqrt(dist);
    }

//    @Override
    public double steer(double[] a, double[] b, double dist) {
//        double d = Math.min(dist, _eta);
//
//        for (int i=0 ; i<_dimensions ; i+=2) {
//            double dx = b[i] - a[i];
//            double dy = b[i+1] - a[i+1];
//
//            double s = Math.sqrt(dx*dx + dy*dy);
//            b[i] = a[i] + dx * d / s;
//            b[i+1] = a[i+1] + dy * d / s;
//        }
//
//        double result = dist(a, b);
//        assert result < _eta+1e-6;
//        assert d < result+1e-6 && result < d+1e-6;
//
//        return d;

        if (dist < _eta) {
            return dist;
        } else {
            double scale = _eta / dist;
            for (int i=0 ; i<_dimensions ; ++i) {
                b[i] = a[i] + (b[i] - a[i]) * scale;
            }
            return _eta;
        }
    }

    @Override
    public boolean clear(double[] config) {
        // robot-robot collision
        for (int j=0 ; j<_dimensions ; j+=2) {
            for (int k=j+2 ; k<_dimensions ; k+=2) {
                double rdx = config[j] - config[k];
                double rdy = config[j+1] - config[k+1];
                if (rdx*rdx + rdy*rdy <= ROBOT_RADIUS_X2POW2) {
//                        _log.debug("robot-robot collision between "+(j/2)+" and "+(j/2));
                    return false;
                }
            }
        }
        // robot-obstacle collision
        for (Obstacle obstacle : _obstacles) {
            for (int j=0 ; j<_dimensions ; j+=2) {
//                    if (obstacle.intersects(p[j], p[j+1], ROBOT_RADIUS, ROBOT_RADIUS)) {
                if (obstacle.distToPoint(config[j], config[j+1]) < ROBOT_RADIUS) {
//                        _log.debug("robot-obstacle collision "+j/2+" and "+obstacle);
                    return false;
                }
            }
        }
        return true;
    }


    @Override
    public boolean link(double[] a, double[] b) {
//        _log.info(
//            String.format(
//                "link{(%f,%f,%f,%f,%f,%f) - (%f,%f,%f,%f,%f,%f)} (dist=%f)",
//                a[0], a[1], a[2], a[3], a[4], a[5],
//                b[0], b[1], b[2], b[3], b[4], b[5],
//                dist(a, b)));

        double[] dx = new double[_dimensions];
        double dist = 0;
        for (int i=0 ; i<_dimensions ; ++i) {
            dx[i] = b[i] - a[i];
            dist += dx[i] * dx[i];
        }

        dist = Math.sqrt(dist);

        int steps = (int)Math.floor(dist / DISCRETIZATION) + 2;

        double[] p = new double[_dimensions];

        for (int i=0 ; i<=steps ; ++i) {
            for (int j=0 ; j<_dimensions ; ++j) {
                p[j] = (a[j] * (steps - i) + b[j] * i) / steps; // - ROBOT_RADIUS/2;
            }
            if (!clear(p)) {
                return false;
            }
        }

        return true;
    }

    @Override
    public boolean goal(double[] conf) {
        if (GOAL_BIASED) {
            return link(conf, _goal);
        } else {
            return dist(conf, _goal) < GOAL_RADIUS;
        }
    }

    public Obstacle[] obstacles() {
        return _obstacles;
    }
}

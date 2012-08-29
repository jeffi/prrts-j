package edu.unc.robotics.prrts;

/**
 * RobotModel
 *
 * @author jeffi
 */
public interface RobotModel {
    boolean goal(double[] config);
    boolean clear(double[] config);
    boolean link(double[] a, double[] b);
}

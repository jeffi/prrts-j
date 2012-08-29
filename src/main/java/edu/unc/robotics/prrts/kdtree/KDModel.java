package edu.unc.robotics.prrts.kdtree;

/**
 * KDModel
 *
 * @author jeffi
 */
public interface KDModel {
    int dimensions();
    void getBounds(double[] min, double[] max);
    double dist(double[] config, double[] target);
}

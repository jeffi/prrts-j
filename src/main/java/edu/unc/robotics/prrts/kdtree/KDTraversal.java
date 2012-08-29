package edu.unc.robotics.prrts.kdtree;

/**
 * KDTraversal
 *
 * @author jeffi
 */
public interface KDTraversal<V> {
    double distToLastNearest();
    void insert(double[] config, V value);
    V nearest(double[] target);
    int near(double[] target, double radius, KDNearCallback<V> callback);
}

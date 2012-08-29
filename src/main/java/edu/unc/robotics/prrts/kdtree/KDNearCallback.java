package edu.unc.robotics.prrts.kdtree;

/**
 * KDNearCallback
 *
 * @author jeffi
 */
public interface KDNearCallback<V> {
    void kdNear(double[] target, int index, double[] config, V value, double dist);
}

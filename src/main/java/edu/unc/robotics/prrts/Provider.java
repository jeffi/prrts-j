package edu.unc.robotics.prrts;

/**
 * Equivalent of Guice's.
 *
 * @author jeffi
 */
public interface Provider<V> {
    V get();
}

package edu.unc.robotics.prrts;

/**
 * SingletonProvider
 *
 * @author jeffi
 */
public class SingletonProvider<V> implements Provider<V> {
    private final V _singleton;

    public SingletonProvider(V singleton) {
        _singleton = singleton;
    }

    @Override
    public V get() {
        return _singleton;
    }
}

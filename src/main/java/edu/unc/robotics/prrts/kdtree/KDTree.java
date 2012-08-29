package edu.unc.robotics.prrts.kdtree;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReferenceFieldUpdater;
import java.util.logging.Logger;

/**
 * KDTree
 *
 * @author jeffi
 */
public class KDTree<V> {
    private static Logger _log = Logger.getLogger(KDTree.class.getName());

    static final class Node<V> {
        final double[] config;
        final V value;
        volatile Node<V> a;
        volatile Node<V> b;

        static final AtomicReferenceFieldUpdater<Node,Node> A_UPDATER =
            AtomicReferenceFieldUpdater.newUpdater(Node.class, Node.class, "a");
        static final AtomicReferenceFieldUpdater<Node,Node> B_UPDATER =
            AtomicReferenceFieldUpdater.newUpdater(Node.class, Node.class, "b");

        Node(double[] c, V v) {
            assert c != null && v != null;
            config = c;
            value = v;
        }

        boolean setA(Node<V> old, Node<V> n) {
            return A_UPDATER.compareAndSet(this, old, n);
        }

        boolean setB(Node<V> old, Node<V> n) {
            return B_UPDATER.compareAndSet(this, old, n);
        }

        @SuppressWarnings("unchecked")
        public Node<V> getA() {
            return A_UPDATER.get(this);
        }

        @SuppressWarnings("unchecked")
        public Node<V> getB() {
            return B_UPDATER.get(this);
        }
    }

    final KDModel _model;
    final int _dimensions;
    final Node<V> _root;

    public KDTree(KDModel model, double[] rootConfig, V rootValue) {
        _model = model;
        _dimensions = model.dimensions();
        _root = new Node<V>(rootConfig, rootValue);
    }

    public Iterable<V> values() {
        List<V> list = new ArrayList<V>();
        buildList(list, _root);
        return list;
    }

    private void buildList(List<V> list, Node<V> node) {
        if (node != null) {
            list.add(node.value);
            buildList(list, node.a);
            buildList(list, node.b);
        }
    }

    public KDTraversal<V> newTraversal() {
        return new Traversal();
    }

    class Traversal implements KDTraversal<V> {
        private final int _dimensions = KDTree.this._dimensions;
        private final double[] _min = new double[_dimensions];
        private final double[] _max = new double[_dimensions];

        int _statConcurrentInserts;

        public double _dist;
        public V _nearest;

        @Override
        public double distToLastNearest() {
            return _dist;
        }

        @Override
        public void insert(double[] config, V value) {
            double[] min = _min;
            double[] max = _max;

            _model.getBounds(min, max);

            Node<V> newNode = new Node<V>(config, value);
            Node<V> n = _root;
            int depth = 0;

            for (;; ++depth) {
                int axis = depth % _dimensions;
                double mp = (min[axis] + max[axis]) / 2;
                double v = config[axis];

                if (v < mp) {
                    // a-side
                    if (n.getA() == null) {
                        if (n.setA(null, newNode)) {
                            break;
                        }
                        _statConcurrentInserts++;
                    }
                    _max[axis] = mp;
                    n = n.getA();
                } else {
                    // b-side
                    if (n.getB() == null) {
                        if (n.setB(null, newNode)) {
                            break;
                        }
                        _statConcurrentInserts++;
                    }
                    _min[axis] = mp;
                    n = n.getB();
                }
            }
        }

        public V nearest(double[] target) {
            _dist = Double.MAX_VALUE;
            _model.getBounds(_min, _max);
            nearest(_root, target, 0);
            return _nearest;
        }

        private void nearest(Node<V> n, double[] target, int depth) {
            final int axis = depth % _dimensions;
            final double d = _model.dist(n.config, target);

            if (d < _dist) {
                _dist = d;
                _nearest = n.value;
            }

            final double mp = (_min[axis] + _max[axis]) / 2;

            if (target[axis] < mp) {
                // a-side
                Node<V> a = n.getA();
                if (a != null) {
                    double tmp = _max[axis];
                    _max[axis] = mp;
                    nearest(a, target, depth + 1);
                    _max[axis] = tmp;
                }

                Node<V> b = n.getB();
                if (b != null) {
                    double tmp = Math.abs(mp - target[axis]);
                    if (tmp < _dist) {
                        tmp = _min[axis];
                        _min[axis] = mp;
                        nearest(b, target, depth + 1);
                        _min[axis] = tmp;
                    }
                }
            } else {
                // b-side
                Node<V> b = n.getB();
                if (b != null) {
                    double tmp = _min[axis];
                    _min[axis] = mp;
                    nearest(b, target, depth + 1);
                    _min[axis] = tmp;
                }

                Node<V> a = n.getA();
                if (a != null) {
                    double tmp = Math.abs(mp - target[axis]);
                    if (tmp < _dist) {
                        tmp = _max[axis];
                        _max[axis] = mp;
                        nearest(a, target, depth + 1);
                        _max[axis] = tmp;
                    }
                }
            }
        }

        @Override
        public int near(double[] target, double radius, KDNearCallback<V> callback) {
            _model.getBounds(_min, _max);
            return near(_root, target, radius, callback, 0, 0);
        }

        private int near(Node<V> n, double[] target, double radius, KDNearCallback<V> callback, int count, int depth) {
            final double d = _model.dist(n.config, target);
            if (d < radius) {
                callback.kdNear(target, count++, n.config, n.value, d);
            }
            final int axis = depth % _dimensions;
            final double mp = (_min[axis] + _max[axis]) / 2;
            final double dm = Math.abs(mp - target[axis]);

            Node<V> a = n.getA();

            if (a != null && (target[axis] < mp || dm < radius)) {
                // in or near a-side
                double tmp = _max[axis];
                _max[axis] = mp;
                count = near(a, target, radius, callback, count, depth + 1);
                _max[axis] = tmp;
            }

            Node<V> b = n.getB();

            if (b != null && (mp <= target[axis] || dm < radius)) {
                // in or near b-side
                double tmp = _min[axis];
                _min[axis] = mp;
                count = near(b, target, radius, callback, count, depth + 1);
                _min[axis] = tmp;
            }

            return count;
        }
    }
}

package edu.unc.robotics.prrts;

import edu.unc.robotics.prrts.kdtree.KDModel;
import edu.unc.robotics.prrts.kdtree.KDNearCallback;
import edu.unc.robotics.prrts.kdtree.KDTraversal;
import edu.unc.robotics.prrts.kdtree.KDTree;
import edu.unc.robotics.prrts.util.MersenneTwister;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicReferenceFieldUpdater;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * PRRTStar
 *
 * @author jeffi
 */
public class PRRTStar {
    private static final Logger _log = Logger.getLogger(PRRTStar.class.getName());

    private static final int INITIAL_NEAR_LIST_CAPACITY = 1024;

    // Robotic System
    KDModel _kdModel;
    Provider<RobotModel> _robotModelProvider;
    Provider<Random> _randomProvider = new Provider<Random>() {
        @Override
        public Random get() {
            return new MersenneTwister();
        }
    };

    // RRT* Parameters
    double[] _startConfig;
    double _gamma = 5.0;
    boolean _perThreadRegionSampling = true;
    int _regionSplitAxis = 0;
    int _samplesPerStep = 1;
    double[] _targetConfig;

    // Run duration
    int _threadCount;
    long _timeLimit;
    long _startTime;
    int _sampleLimit;

    // Runtime data
    KDTree<Node> _kdTree;
    final AtomicInteger _stepNo = new AtomicInteger(0);
    final AtomicBoolean _done = new AtomicBoolean(false);
    CountDownLatch _doneLatch;
    final AtomicReference<Link> _bestPath = new AtomicReference<Link>();

    public PRRTStar(KDModel kdModel, Provider<RobotModel> robotModelProvider, double[] init) {
        _kdModel = kdModel;
        _robotModelProvider = robotModelProvider;
        _startConfig = init;
        _kdTree = new KDTree<Node>(kdModel, init, new Node(init, false));
    }

    /**
     * Sets the gamma value for RRT*.  The default value is 5.
     *
     * @param gamma the value to set.
     */
    public void setGamma(double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _gamma = gamma;
    }

    /**
     * Enables per-thread region based sampling.  The default value is true.
     *
     * @param b
     */
    public void setPerThreadRegionSampling(boolean b) {
        _perThreadRegionSampling = b;
    }

    /**
     * Sets a pseudo-random number generator provider.  The provider is asked
     * to provide random number generators for each of the threads at runtime.
     *
     * @param randomProvider
     */
    public void setRandomProvider(Provider<Random> randomProvider) {
        _randomProvider = randomProvider;
    }

    /**
     * Returns the current step no.  May be called while running, in which
     * case the value returned will be less than or equal to the actual
     * step no.  Called after running, this may return more than the requested
     * number of samples since multiple threads may finish concurrently.
     *
     * @return the approximate step number.
     */
    public int getStepNo() {
        return _stepNo.get();
    }

    public Iterable<Node> getNodes() {
        return _kdTree.values();
    }

    /**
     * Represents a single configuration in the RRT* tree.  The path to the
     * node can be computed by following the parents until null, and then
     * reversing the order.  This class is part of the public API, but is
     * also used internally.  The package-private members are intentionally
     * not part of the public API as they are subject to change.
     *
     * The public API may safely be accessed while the PRRTStar is running.
     * There is a possibility that the path to a node will change while it
     * is being accessed, but the config member will not change.  For
     * efficiency, the config member is exposed as a direct reference an array.
     * It should NOT be modified by the caller.
     */
    public static class Node {
        final double[] config;
        final boolean inGoal;

        volatile Link link;

        private static final AtomicReferenceFieldUpdater<Node,Link> LINK =
            AtomicReferenceFieldUpdater.newUpdater(Node.class, Link.class, "link");

        Node(double[] config, boolean inGoal) {
            this.config = config;
            this.inGoal = inGoal;
            this.link = new Link(this);
        }

        Node(double[] config, boolean inGoal, double linkDist, Link parent) {
            this.config = config;
            this.inGoal = inGoal;

            Link link = new Link(this, linkDist, parent);

            this.link = link;
            parent.addChild(link);
        }

        Link setLink(Link oldLink, double linkDist, Link parent) {
            Link newLink = new Link(this, linkDist, parent);

            if (!LINK.compareAndSet(this, oldLink, newLink)) {
                return null;
            }

            assert newLink.pathDist <= oldLink.pathDist;
            parent.addChild(newLink);
            return newLink;
        }

        /**
         * Returns the configuration of this node in the RRT* tree.  The
         * returned value is a direct reference to an array (not a copy)
         * and thus should NOT be modified by the caller.
         *
         * @return the configuration
         */
        public double[] getConfig() {
            return config;
        }

        /**
         * Returns the parent of this configuration.  It is the best known
         * path to this configuration as of the time it is called.  The
         * returned value may change while PRRT* is running.  If null is
         * returned, the node represents the initial configuration.  There
         * are no provisions to return the node's children.
         *
         * @return the nodes parent, or null if this is the root node.
         */
        public Node getParent() {
            Link parent = this.link.parent;
            return parent == null ? null : parent.node;
        }
    }

    static class Link {
        final Node node;
        final double linkDist;
        final double pathDist;

        volatile Link parent;
        volatile Link firstChild;
        volatile Link nextSibling;

        private static final AtomicReferenceFieldUpdater<Link,Link> PARENT =
            AtomicReferenceFieldUpdater.newUpdater(Link.class, Link.class, "parent");
        private static final AtomicReferenceFieldUpdater<Link,Link> FIRST_CHILD =
            AtomicReferenceFieldUpdater.newUpdater(Link.class, Link.class, "firstChild");
        private static final AtomicReferenceFieldUpdater<Link,Link> NEXT_SIBLING =
            AtomicReferenceFieldUpdater.newUpdater(Link.class, Link.class, "nextSibling");

        public Link(Node root) {
            this.node = root;
            this.linkDist = 0;
            this.pathDist = 0;
            this.parent = null;
        }

        public Link(Node node, double linkDist, Link parent) {
            this.node = node;
            this.linkDist = linkDist;
            this.pathDist = parent.pathDist + linkDist;
            this.parent = parent;
        }

        boolean setParent(Link oldValue, Link newValue) {
            return PARENT.compareAndSet(this, oldValue, newValue);
        }

        boolean setFirstChild(Link oldValue, Link newValue) {
            return FIRST_CHILD.compareAndSet(this, oldValue, newValue);
        }

        boolean setNextSibling(Link oldValue, Link newValue) {
            return NEXT_SIBLING.compareAndSet(this, oldValue, newValue);
        }

        void addChild(Link child) {
            Link expected = null;
            Link nextSibling;

            do {
                nextSibling = firstChild;

                if (!child.setNextSibling(expected, nextSibling)) {
                    assert false : "nextSibling initialized to unexpected value";
                }

                expected = nextSibling;
            } while (!setFirstChild(nextSibling, child));
        }

        public boolean isExpired() {
            return node.link != this;
        }

        public Link removeFirstChild() {
            Link child;
            Link sibling;

            do {
                child = firstChild;
                if (child == null) {
                    return null;
                }
                sibling = child.nextSibling;
            } while (!setFirstChild(child, sibling));

            if (!child.setNextSibling(sibling, null)) {
                assert false : "sibling changed after removal";
            }

            return child;
        }

        public boolean removeChild(final Link child) {
            Link sibling;
            Link n;
            Link p;

            assert child.isExpired() : "removing unexpired child";
            assert child.parent == this : "not child's parent";

        outer:
            for (;;) {
                n = firstChild;

                if (n == child) {
                    sibling = child.nextSibling;

                    if (setFirstChild(child, sibling)) {
                        break;
                    } else {
                        continue;
                    }
                }

                if (n == null) {
                    return false;
                }

                for (;;) {
                    p = n;

                    n = n.nextSibling;

                    if (n == null) {
                        return false;
                    }

                    if (n == child) {
                        sibling = child.nextSibling;


                        // TODO: double check this logic.  could the child
                        // now be the first element in the list?

                        if (p.setNextSibling(child, sibling)) {
                            break outer;
                        } else {
                            break;
                        }
                    }
                }
            }

            child.setNextSibling(sibling, null);

            return true;
        }
    }

    /**
     * Returns the best path found so far.  This method is safe to be called
     * while PRRT* is running.
     *
     * @return the best path, or null if none found so far.
     */
    public Path getBestPath() {
        Link link = _bestPath.get();
        if (link == null) {
            return null;
        }
        List<double[]> configs = new LinkedList<double[]>();
        double pathDist = link.pathDist;
        if (!link.node.inGoal) {
            assert _targetConfig != null;
            configs.add(_targetConfig);
            pathDist += _kdModel.dist(link.node.config, _targetConfig);
        }
        for ( ; link != null ; link = link.parent) {
            configs.add(link.node.config);
        }
        Collections.reverse(configs);
        return new Path(pathDist, configs);
    }

    public Path runForDuration(int threadCount, long milliseconds) {
        return runForDuration(threadCount, milliseconds, TimeUnit.MILLISECONDS);
    }

    public Path runForDuration(int threadCount, long duration, TimeUnit timeUnit) {
        if (duration <= 0) {
            throw new IllegalArgumentException("invalid duration, must be > 0");
        }
        return run(threadCount, Integer.MAX_VALUE, duration, timeUnit);
    }

    public Path runSamples(int threadCount, int samples) {
        return run(threadCount, samples, 0, null);
    }

    public Path runIndefinitely(int threadCount) {
        return run(threadCount, Integer.MAX_VALUE, 0, null);
    }

    private Path run(int threadCount, int sampleLimit, long duration, TimeUnit timeUnit) {
        if (threadCount < 1) {
            throw new IllegalArgumentException("thread count must be >= 1");
        }

        int availableProcessors = Runtime.getRuntime().availableProcessors();

        if (threadCount > availableProcessors) {
            _log.warning(String.format(
                "Thread count (%d) exceeds available processors (%d)",
                threadCount, availableProcessors));
        }

        int dimensions = _kdModel.dimensions();

        _doneLatch = new CountDownLatch(threadCount);
        _sampleLimit = sampleLimit;
        _timeLimit = duration > 0 ? timeUnit.toNanos(duration) : 0;

        _startTime = System.nanoTime();

        Worker[] workers = new Worker[threadCount];
        for (int i=0 ; i<threadCount ; ++i) {
            workers[i] = new Worker(i, threadCount);
        }

        ThreadGroup threadGroup = Thread.currentThread().getThreadGroup();
        Thread[] threads = new Thread[threadCount];
        for (int i=1 ; i<threadCount ; ++i) {
            threads[i] = new Thread(threadGroup, workers[i]);
            threads[i].start();
        }

        // worker 0 runs on the calling thread (thus if only 1 thread is
        // specified, no additional threads are created)
        workers[0].run();

        // We could join all worker threads here, but a latch will allow
        // the calling thread to continue sooner.
        // (TODO: test difference on multiple OSs)
        try {
            if (!_doneLatch.await(1, TimeUnit.SECONDS)) {
                _log.warning("waiting too long for workers, trying to join");
                for (int i=1 ; i<threadCount ; ++i) {
                    threads[i].join();
                }
            }
        } catch (InterruptedException e) {
            _log.log(Level.WARNING, "Interrupted", e);
        }

        return getBestPath();
    }

    static class NearNode implements Comparable<NearNode> {
        Link link;
        double linkDist;
        double pathDist;

        @Override
        public int compareTo(NearNode o) {
            return Double.compare(this.pathDist, o.pathDist);
        }
    }

    class Worker implements Runnable, KDNearCallback<Node> {
        final int _dimensions = _kdModel.dimensions();
        final int _workerNo;
        final int _threadCount;

        KDTraversal<Node> _kdTraversal;
        RobotModel _robotModel;
        double[] _sampleMin;
        double[] _sampleMax;
        Random _random;

        NearNode[] _nearList = new NearNode[INITIAL_NEAR_LIST_CAPACITY];

        public Worker(int workerNo, int threadCount) {
            _workerNo = workerNo;
            _threadCount = threadCount;
        }

        void updateBestPath(Link link, double radius) {
            Link currentBestPath;
            double distToGoal;
            Node node = link.node;

            if (node.inGoal) {
                distToGoal = link.pathDist;
            } else if (_targetConfig != null) {
                // searching for a configuration that can reach a pre-specified
                // target configuration

                double distToTarget = _kdModel.dist(node.config, _targetConfig);
                if (distToTarget > radius ||
                    !_robotModel.link(node.config, _targetConfig))
                {
                    return;
                }
                distToGoal = link.pathDist + distToTarget;
            } else {
                return;
            }

            do {
                currentBestPath = _bestPath.get();

                if (currentBestPath != null) {
                    double bestDist = currentBestPath.pathDist;

                    if (!currentBestPath.node.inGoal) {
                        // current best is not a goal itself, so it must
                        // link to a target configuration
                        bestDist += _kdModel.dist(currentBestPath.node.config, _targetConfig);
                        // TODO: dist will be called here on every update,
                        // it might be worth caching
                    }

                    if (distToGoal >= bestDist) {
                        return;
                    }
                }
            } while (!_bestPath.compareAndSet(currentBestPath, link));
        }

        private void updateChildren(Link newParent, Link oldParent, double radius) {
            assert newParent.node == oldParent.node : "updating links of different nodes";
            assert oldParent.isExpired() : "updating non-expired link";
            assert newParent.pathDist <= oldParent.pathDist : "updating to longer path";

            for (;;) {
                Link oldChild = oldParent.removeFirstChild();

                if (oldChild == null) {
                    // done.

                    if (newParent.isExpired()) {
                        oldParent = newParent;
                        newParent = oldParent.node.link;
                        continue;
                    }

                    return;
                }

                if (oldChild.isExpired()) {
                    // TODO: increment concurrent rewiring stat
                    assert _threadCount > 1;
                    continue;
                }

                double pathDist = newParent.pathDist + oldChild.linkDist;

                Node node = oldChild.node;

                if (node.link.parent.node != oldParent.node) {
                    continue;
                }

                Link newChild = node.setLink(oldChild, oldChild.linkDist, newParent);

                if (newChild != null) {
                    // TODO: increment updated children count
                    updateChildren(newChild, oldChild, radius);
                    updateBestPath(newChild, radius);
                } else {
                    // TODO: increment concurrent rewirings stat
                    assert _threadCount > 1;
                    assert node.link != oldChild;
                }
            }
        }

        private void rewire(Link oldLink, double linkDist, Node newParent, double radius) {
            assert oldLink.parent != null;
            assert oldLink.parent.node != newParent;

            Node node = oldLink.node;

            Link parentLink = newParent.link;

            double pathDist = parentLink.pathDist + linkDist;

            // check if rewiring would create a shorter path
            if (pathDist >= oldLink.pathDist) {
                return;
            }

            // check if rewiring is possible
            if (!_robotModel.link(oldLink.node.config,  newParent.config)) {
                return;
            }

            // rewire the node.  this loop continues to attempt atomic
            // updates until either the update succeeds or the pathDist
            // of the oldLink is found to be better than what we're trying
            // to put in
            do {
                Link newLink = node.setLink(oldLink, linkDist, parentLink);

                if (newLink != null) {
                    updateChildren(newLink, oldLink, radius);
                    updateBestPath(newLink, radius);

                    if (parentLink.isExpired()) {
                        updateChildren(parentLink.node.link, parentLink, radius);
                    }

                    // Setting newLink expires oldLink but doesn not remove
                    // it from its parent.  Here we do a little cleanup.
                    // We do it after the expired parent check since the parent
                    // will likely have already cleaned this up, and this call
                    // will be O(1) instead of O(n)
                    if (!oldLink.parent.removeChild(oldLink)) {
                        assert _threadCount > 1 : "concurrent update running with 1 thread";
                    }

                    return;
                }

                // TODO: increment concurrent rewiring stat

                assert _threadCount > 1 : "concurrent update running with 1 thread";

                Link updatedOldLink = node.link;

                assert updatedOldLink != oldLink;

                oldLink = updatedOldLink;

            } while (pathDist < oldLink.pathDist);
        }

        /**
         * Callback handler for calls to kdNear.  This method adds near nodes
         * to the worker's nearList.
         *
         * @param target
         * @param index
         * @param config
         * @param value
         * @param dist
         */
        @Override
        public void kdNear(double[] target, int index, double[] config, Node value, double dist) {
            if (index == _nearList.length) {
                _nearList = Arrays.copyOf(_nearList, index * 2);
            }

            NearNode n = _nearList[index];

            if (n == null) {
                _nearList[index] = n = new NearNode();
            }

            n.link = value.link;
            n.linkDist = dist;
            n.pathDist = n.link.pathDist + dist;
        }

        /**
         * Checks if a configuration is a goal configuration.  If we are
         * searching for a path to a target goal configuration, this method
         * always returns false, since we will check the configuration can
         * extend to the target later.
         *
         * @param config the configuration to test
         * @return true if the configuration is in the goal region
         */
        private boolean inGoal(double[] config) {
            return _targetConfig == null && _robotModel.goal(config);
        }

        private void randomize(double[] config) {
            for (int i=_dimensions ; --i >= 0 ; ) {
                config[i] = _random.nextDouble() * (_sampleMax[i] - _sampleMin[i])
                    + _sampleMin[i];
            }
        }

        private void steer(double[] newConfig, double[] nearConfig, double t) {
            for (int i=_dimensions ; --i >= 0 ; ) {
                newConfig[i] = nearConfig[i] + (newConfig[i] - nearConfig[i]) * t;
            }
        }

        private boolean step(int stepNo, double[] newConfig) {
            // generate a new random sample
            randomize(newConfig);

            if (!_robotModel.clear(newConfig)) {
                return false;
            }

            double radius = _gamma * Math.pow(
                Math.log(stepNo + 1) / (stepNo + 1),
                1.0 / _dimensions);

            int nearCount = _kdTraversal.near(newConfig, radius, this);

            if (nearCount == 0) {
                // nothing with in radius
                Node nearest = _kdTraversal.nearest(newConfig);
                double distToNearest = _kdTraversal.distToLastNearest();

                assert radius < distToNearest;

                steer(newConfig, nearest.config, radius / distToNearest);

                if (!_robotModel.clear(newConfig)) {
                    return false;
                }

                if (!_robotModel.link(nearest.config, newConfig)) {
                    return false;
                }

                // This should be radius, but might be off slightly so we
                // recalculate just to be safe.
                distToNearest = _kdModel.dist(newConfig, nearest.config);

                Node newNode = new Node(
                    newConfig, inGoal(newConfig), distToNearest, nearest.link);

                updateBestPath(newNode.link, radius);

                _kdTraversal.insert(newConfig, newNode);
                return true;
            }

            // Sort the array from nearest to farthest.  After sorting
            // we can traverse the array sequentially and select the first
            // configuration that can link.  We know that anything after it
            // in the array will be further away, and thus potentially save
            // a lot of calls to the (usually) expensive link() method.
            Arrays.sort(_nearList, 0, nearCount);

            for (int i=0 ; i<nearCount ; ++i) {
                Link link = _nearList[i].link;

                if (!_robotModel.link(link.node.config, newConfig)) {
                    // help GC
                    _nearList[i].link = null;
                    continue;
                }

                // Found a linkable configuration.  Create the node
                // and link it in here.

                Node newNode = new Node(
                    newConfig, inGoal(newConfig), _nearList[i].linkDist, link);

                updateBestPath(newNode.link, radius);

                // Put the node in the KD-Tree.  After insertion,
                // other threads will "see" the new node and may start
                // rewiring it.

                _kdTraversal.insert(newConfig, newNode);

                // help GC
                _nearList[i].link = null;

                // For the remaining nodes in the near list, rewire
                // their links to go through the newly inserted node
                // if doing so is feasible and would shorten their path
                //
                // We go through the remaining list in reverse order to
                // reduce the number of rewirings we do on the farther nodes.
                // If we went from nearest to farthest, the far nodes might
                // rewire through the near nodes, then through the newly added
                // node.
                for (int j=nearCount ; --j > i ; ) {
                    rewire(_nearList[j].link, _nearList[j].linkDist, newNode, radius);

                    // help GC
                    _nearList[j].link = null;
                }

                return true;
            }

            // if we're here, we've looped through the entire near list and
            // found no nodes that can be linked through.  We return false
            // indicating we failed to add a node.
            return false;
        }


        private void generateSamples() {
            // only have 1 worker check the time limit
            final boolean checkTimeLimit = (_workerNo == 0) && (_timeLimit > 0);
            double[] newConfig = new double[_dimensions];
            int stepNo = _stepNo.get();

            while (!_done.get()) {
                if (step(stepNo, newConfig)) {
                    stepNo = _stepNo.incrementAndGet();
                    if (stepNo > _sampleLimit) {
                        _done.set(true);
                    }
                    // sample was added, create a new one
                    newConfig = new double[_dimensions];
                } else {
                    // failed add a sample, refresh the step no
                    // and try again
                    stepNo = _stepNo.get();
                }

                if (checkTimeLimit) {
                    long now = System.nanoTime();
                    if (now - _startTime > _timeLimit) {
                        _done.set(true);
                    }
                }
            }
        }

        public void run() {
            try {
                // worker initialization (normally would occur in constructor
                // but put here it will be run on its own thread in parallel)
                _robotModel = _robotModelProvider.get();
                _sampleMin = new double[_kdModel.dimensions()];
                _sampleMax = new double[_kdModel.dimensions()];

                _kdModel.getBounds(_sampleMin, _sampleMax);

                if (_perThreadRegionSampling) {
                    double min = _sampleMin[_regionSplitAxis];
                    double t = (_sampleMax[_regionSplitAxis] - min) / _threadCount;
                    _sampleMin[_regionSplitAxis] = min + _workerNo * t;
                    if (_workerNo+1 < _threadCount) {
                        _sampleMax[_regionSplitAxis] = min + (_workerNo + 1) * t;
                    }
                }

                _kdTraversal = _kdTree.newTraversal();
                _random = _randomProvider.get();

                generateSamples();

            } finally {
                _doneLatch.countDown();
            }
        }
    }

}

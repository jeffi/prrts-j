package edu.unc.robotics.prrts;

import edu.unc.robotics.prrts.kdtree.KDModel;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

/**
 * Represent a computed path, the result of an PRRTStar run.
 *
 * @author jeffi
 */
public class Path implements Comparable<Path> {

    /**
     * The distance of the computed path
     */
    public double dist;

    /**
     * The configurations of the path.
     */
    public List<double[]> configs;

    public Path(double dist, List<double[]> configs) {
        this.dist = dist;
        this.configs = configs;
    }

    /**
     * Paths are comparable by their distances.
     *
     * @param that the path to compare to
     * @return -1 if this path is shorter, 0 is equal in length, 1 if longer
     * than the argument path.
     */
    @Override
    public int compareTo(Path that) {
        return Double.compare(this.dist, that.dist);
    }

    public void interpolate(double[] outConfig, double offset, KDModel kdModel) {
        Iterator<double[]> pathIter = configs.iterator();
        double[] from = pathIter.next();
        double[] dest = pathIter.next();
        double distToFrom = 0;
        double distToDest = kdModel.dist(from, dest);

        while (pathIter.hasNext() && distToDest < offset) {
            from = dest;
            distToFrom = distToDest;
            dest = pathIter.next();
            distToDest += kdModel.dist(from, dest);
        }
//        assert distToFrom <= offset && offset <= distToDest : String.format("%f <= %f <= %f", distToFrom, offset, distToDest);
        double distBetween = distToDest - distToFrom;
        offset -= distToFrom;
        offset /= distBetween;

        double s = offset;
        double p = 1.0 - s;

        for (int i=0 ; i<kdModel.dimensions() ; ++i) {
            outConfig[i] = from[i] * p + dest[i] * s;
        }
    }

    public static boolean isBetter(Path a, Path b) {
        if (a == null) {
            return false;
        }
        if (b == null) {
            return true;
        }
        return a.dist < b.dist;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Path)) return false;

        Path path = (Path) o;

        if (Double.compare(path.dist, dist) != 0) return false;
        if (configs.size() != path.configs.size()) return false;
        Iterator<double[]> i1 = configs.iterator();
        Iterator<double[]> i2 = path.configs.iterator();
        while (i1.hasNext()) {
            if (!Arrays.equals(i1.next(), i2.next())) {
                 return false;
            }
        }
        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = dist != +0.0d ? Double.doubleToLongBits(dist) : 0L;
        result = (int) (temp ^ (temp >>> 32));
        for (double[] config : configs) {
            result = 31 * result + Arrays.hashCode(config);
        }
        return result;
    }
}

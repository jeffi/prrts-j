package edu.unc.robotics.prrts.example.arena;

import edu.unc.robotics.prrts.PRRTStar;
import edu.unc.robotics.prrts.Path;
import edu.unc.robotics.prrts.example.geom.Obstacle;

import javax.swing.JComponent;
import javax.swing.SwingUtilities;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Iterator;
import java.util.logging.Logger;

/**
 * ArenaView
 *
 * @author jeffi
 */
public class ArenaView extends JComponent {

    private static final Logger _log = Logger.getLogger(ArenaView.class.getName());
    private PRRTStar _rrtStar;
    private HolonomicArena _robotModel;

    private static final Color[] COLORS = new Color[] {
        Color.BLACK, Color.BLUE, Color.MAGENTA, Color.GREEN
    };
//    private KDTree<PRRTStar.Node>.Traversal _traversal;

    private int _backgroundTreeSize = -1;
    private Image _backgroundImage;
    private double _bestPathDist = 1;
    private long _animTime;
    private double _animationDuration = 5.0 * 1000; // 5 seconds
    private Path _bestPath = null;
    private double[] _frameConfig;

    private double GOAL_RADIUS = 0.5;

    private boolean _paintTree = true;

    private NumberFormat _integerFormat = DecimalFormat.getIntegerInstance();

    public ArenaView(HolonomicArena arena, PRRTStar rrtStar) {
        _rrtStar = rrtStar;
        _robotModel = arena;
        _frameConfig = new double[_robotModel.dimensions()];
//        _traversal = _rrtStar.getKdTree().createTraversal();
    }

//    PRRTStar.Link findGoalNode() {
//        PRRTStar.Link bestPath = _rrtStar.getBestPath();
//        if (bestPath == null) {
//            return null;
//        }
//        return bestPath;
//
////        List<RRTStarNode> nearGoalNodes = _rrtStar.getKdTree().near(_robotModel.getGoal(), GOAL_RADIUS, _traversal);
////
////        RRTStarNode n;
////        if (nearGoalNodes.isEmpty()) {
////            _rrtStar.getKdTree().nearest(_robotModel.getGoal(), _traversal);
////            n = _traversal.nearest;
//////            _log.debug("No nodes in goal found, nearest is: "+_traversal.dist+" at "+n.getPathDist());
////        } else {
//////            Collections.sort(nearGoalNodes); // may fail if node value changes mid sort
////            double bestDist = Double.MAX_VALUE;
////            n = null;
////            for (RRTStarNode nearNode : nearGoalNodes) {
////                double dist = nearNode.getPathDist();
////                if (dist < bestDist) {
////                    n = nearNode;
////                    bestDist = dist;
////                }
////            }
//////            _log.debug("Found "+nearGoalNodes.size()+" nodes in goal, nearest is at "+n.getPathDist());
////        }
////
////        return n;
//    }

//    double distToGoal(double[] conf) {
//        return Math.max(0, _robotModel.dist(conf, _robotModel.getGoal()) - GOAL_RADIUS);
//    }

    @Override
    protected void paintComponent(Graphics graphics) {
        doPaint((Graphics2D)graphics, getSize());

        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                repaint();
            }
        });
    }

    public void doPaint(Graphics2D g, Dimension size) {
        HolonomicArena robotModel = _robotModel;
        double[] min = new double[robotModel.dimensions()];
        double[] max = new double[robotModel.dimensions()];
        robotModel.getBounds(min, max);

        Path bestPath = _rrtStar.getBestPath();

//        double distToGoal = distToGoal(n.config);
//        double pathDist = bestPath.pathCost; // n.getPathDist() + distToGoal;

        if (_backgroundImage == null ||
            _backgroundImage.getWidth(null) != size.width ||
            _backgroundImage.getHeight(null) != size.height ||
//            (_paintTree && _backgroundTreeSize != _rrtStar.getKdTree().size()) ||
            Path.isBetter(bestPath, _bestPath))
        {
            createBGImage(min, max, size, bestPath);
//            _log.debug("Goal node path length: "+bestPath.pathDist+" (in goal: "+bestPath.node.inGoal+")");
            _bestPath = bestPath;
            _animTime = System.currentTimeMillis();
        }

        g.drawImage(_backgroundImage, 0, 0, null);

        AffineTransform savedTransform = g.getTransform();
        double scale = setupGraphics(min, max, size, g);


        if (bestPath != null && bestPath.configs.size() > 1) {
            double offset = (System.currentTimeMillis() - _animTime) / _animationDuration;
            offset -= Math.floor(offset); // clamp to 0..1
            offset *= bestPath.dist; // convert to distance along path;

            bestPath.interpolate(_frameConfig, offset, _robotModel);

            Ellipse2D.Double circle = new Ellipse2D.Double(
                0, 0, HolonomicArena.ROBOT_RADIUS*2, HolonomicArena.ROBOT_RADIUS*2);

            int dim = _robotModel.dimensions();

            for (int i=0 ; i<dim ; i+=2) {
                g.setColor(brighter(COLORS[i/2]));
                circle.x = _frameConfig[i] - HolonomicArena.ROBOT_RADIUS;
                circle.y = _frameConfig[i+1] - HolonomicArena.ROBOT_RADIUS;
                g.fill(circle);
            }
        }

        g.setTransform(savedTransform);

        g.setColor(Color.WHITE);
        FontMetrics fm = g.getFontMetrics();
        String count = _integerFormat.format(_rrtStar.getStepNo());
        g.drawString(count, 4, 4 + fm.getAscent());
        g.setColor(Color.BLACK);
        g.drawString(count, 3, 3 + fm.getAscent());
    }

    private void createBGImage(double[] min, double[] max, Dimension size, Path link) {
        _backgroundTreeSize = _rrtStar.getStepNo();
        _backgroundImage = createImage(size.width, size.height);

        Graphics2D g = (Graphics2D)_backgroundImage.getGraphics();
        AffineTransform savedTransform = g.getTransform();

        double scale = setupGraphics(min, max, size, g);
        int dim = _robotModel.dimensions();

        g.setColor(Color.WHITE);
        g.fillRect(0, 0, size.width, size.height);

        if (true) {
            g.setStroke(new BasicStroke(0f));
            g.setColor(new Color(0x8888ff));
            for (Obstacle obstacle : _robotModel.obstacles()) {
                g.fill(obstacle.shape());
            }
        }


        if (_paintTree) {
            renderRRTTree(g);
        }

        renderPaths(link, g, scale);

        g.setTransform(savedTransform);
        g.dispose();
    }

    private void renderRRTTree(Graphics2D g) {
        int dim = _robotModel.dimensions();
        Line2D.Double line = new Line2D.Double();
        int count = 0;

        for (PRRTStar.Node node : _rrtStar.getNodes()) {
            PRRTStar.Node parent = node.getParent();
            if (parent != null) {
                double[] n = node.getConfig();
                double[] p = parent.getConfig();
                for (int i=0 ; i<dim ; i+=2) {
                    g.setColor(COLORS[i/2]);
                    line.setLine(n[i], n[i+1], p[i], p[i+1]);
                    g.draw(line);
                }
                count++;
            }
        }

        _log.fine("Rendered: " + count + " paths");
    }

    private void renderPaths(
        Path link, Graphics2D g, double scale)
    {
        if (link == null) {
            _bestPathDist = 100;
            return;
        }

        int dim = _robotModel.dimensions();
        Line2D.Double line = new Line2D.Double();
        g.setStroke(new BasicStroke((float)(5/scale)));

        if (link.configs.size() > 1) {
            Iterator<double[]> pathIter = link.configs.iterator();
            double[] prev = pathIter.next();
            while (pathIter.hasNext()) {
                double[] curr = pathIter.next();
                for (int i=0 ; i<dim ; i+=2) {
                    g.setColor(brighter(COLORS[i/2]));
                    line.setLine(prev[i], prev[i+1], curr[i], curr[i+1]);
                    g.draw(line);
                }
                prev = curr;
            }
        }
    }

    private double setupGraphics(double[] min, double[] max, Dimension size, Graphics2D g) {
        g.setRenderingHint(
        RenderingHints.KEY_ANTIALIASING,
        RenderingHints.VALUE_ANTIALIAS_ON);

        g.translate(min[0], min[1]);
        double scale = Math.min(
            size.width / (max[0] - min[0]),
            size.height / (max[1] - min[1]));
        g.scale(scale, scale);
        g.setStroke(new BasicStroke((float)(0.5/scale/_robotModel.dimensions())));
        return scale;
    }

    static Color brighter(Color color) {
        float[] hsb = Color.RGBtoHSB(color.getRed(), color.getGreen(), color.getBlue(), null);
        hsb[1] = Math.max(0.0f, hsb[1] - 0.25f);
        hsb[2] = Math.min(1.0f, hsb[2] + 0.25f);
        color = Color.getHSBColor(hsb[0], hsb[1], hsb[2]);
        return color;
    }
}

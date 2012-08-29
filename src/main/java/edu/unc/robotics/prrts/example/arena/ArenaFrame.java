package edu.unc.robotics.prrts.example.arena;

import edu.unc.robotics.prrts.PRRTStar;
import edu.unc.robotics.prrts.RobotModel;
import edu.unc.robotics.prrts.SingletonProvider;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import java.awt.BorderLayout;
import java.lang.reflect.InvocationTargetException;

/**
 * ArenaFrame
 *
 * @author jeffi
 */
public class ArenaFrame extends JFrame {

    public ArenaFrame(HolonomicArena arena, PRRTStar rrtStar) {
        super("RRT*");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new ArenaView(arena, rrtStar));
    }


    public static void main(String[] args) throws InterruptedException, InvocationTargetException {
        final HolonomicArena arena = new HolonomicArena(1);
        double[] init = {7.0, 1.0, 8, 8, 9, 1, 1, 9};

        final PRRTStar rrtStar = new PRRTStar(arena, new SingletonProvider<RobotModel>(arena), init);

        rrtStar.setGamma(10.0);
        rrtStar.setPerThreadRegionSampling(false);
//        rrtStar.setSamplesPerStep(100);

        SwingUtilities.invokeAndWait(new Runnable() {
            @Override
            public void run() {
                ArenaFrame frame = new ArenaFrame(arena, rrtStar);
                frame.setSize(800, 800);
                frame.setVisible(true);

                frame.repaint();
            }
        });

        Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().getThreadGroup().setMaxPriority(Thread.MIN_PRIORITY);

        rrtStar.runForDuration(4, 60000);
    }
}

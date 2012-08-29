Parallel Rapidly-exploring Randomized Trees *
=============================================

This code is an implementation of Parallel RRT*--an asymptoptically
optimal robotic motion planning algorithm.  It utilizes
multi-core/multi-processor SMP threading and atomic operations to
implement a RRT* that scales linearly with the number of cores in use.

To enable super-linear speedup, each thread may be given a partition
of the configuation space to sample.  This approach keeps the
working-set of each thread smaller than the non-partitioned approach,
allowing for more effective use of CPU caches.  It's not without its
downside however, it is unknown how partitioning will effect the
speedup when given certain edge-case robotic systems, such as ones
that would have partitions in a completed obstructed portion of
configuration space.

Note on Turbo Boost:
On some SMP configurations, having fewer cores in use allows the
hardware to run at faster clock speeds (see Intel 'Turbo Boost').  In
this case, linear and super-linear speedup may not be seen, however
there will still be an overall speedup benefit.

Note on Simultaneous Multithreading (SMT)
-----------------------------------------

On some SMP configurations, logical processor threads share
computational cores (e.g. Intel Hyper-Threading), which allows more
threads to run simultenously at the expense of stalling for competing
access to shared computational units.  The net effect is usually that
2 threads on 1 core do not run as fast on 2 threads on 2 cores.  PRRT*
has been tested to work well in the presense of SMT, but the
(super-)linear-speedup is not easily observed or for that matter
computed.  As an example, a 2-core processor might experience a 2.1x
speedup on one simulation with SMT disabled.  Enabling SMT (for a
total of 4 logical threads in this example) and running 4-threads in
parallel might see a 3.2x speedup.  In terms of threads the speedup is
sub-linear, but in terms of cores, we'll... we're doing pretty well.

As a cautionary note however, partitioned sampling may run unevenly
with SMT enabled, since threads will run slower or faster depending on
what other thread is scheduled on the same core.  In our experience it
is best to run with all available threads (e.g. 4-core w/ 2x SMT
should use 8 threads), in the presense of SMT when using partitioned
sampling.

Tested implementation:
----------------------

This implementation has been tested on a Intel and AMD x86_64-based
SMP machines in various configurations.  The memory model of the these
machines is strong enough to enable certain assumptions about the
memory access patterns.  This implementation may not run well on other
SMP architectures.  If you are interested in testing/porting the code
to run on another architecture, please contact the authors.


BUILDING
--------

This project requires Maven 2 or better (tested on Maven 3, but should
work fine on 2).  It makes uses of Java 6 language features, but
should be easily ported to Java 5 if required (e.g. remove @Overrides
on interface methods).

To build the jar, run:

    % mvn package

To run an example 2-D holonomic disc robot simulation, run:

    % java -cp target/prrts-1.0.jar edu.unc.robotics.prrts.example.arena.ArenaFrame

It will run for 6 seconds or 4 threads, and then show the generated
motion plan in action.  To terminate, close the window or hit CTRL-C
on the command line.  To tweak the parameters, edit the main method in
ArenaFrame.

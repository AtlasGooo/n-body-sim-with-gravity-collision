import edu.princeton.cs.algs4.*;

import java.awt.Color;
import java.util.PriorityQueue;


/**
 * The 2D-N-Body Elastic collision simulator class. The class provides a
 * relatively efficient API for any simulations the client might want to do with
 * elastic 2-D bodies. It deals with the events occurring during simulation.
 * It uses the {@code Body} data type to achieve this goal. Events are
 * scheduled for each possible collisions and are processed in the chronological
 * order.
 * 
 */
public class  IncrementEvent {

    private double HZ = 0.5;    // redraw frequency as redraws per clock tick (in Simulator time)

    private PriorityQueue<Event> pq;    // the event priority queue
    private Body[] bodies;       // the array of bodies
    private double t = 0.0;                   // simulation clock time

    // (lzj) add map radius xmin,xmax,ymin,ymax
    private final double xmin,xmax,ymin,ymax;

    /**
     * Constructs the simulator class with given array of bodies.
     *
     * @param bodies the array of bodies
     */
    public IncrementEvent(Body[] bodies, double xmin, double xmax, double ymin, double ymax) {
        // make a defensive copy to support immuatability


        /**
         * (lzj) TODO: attention here !
         * the bodies should be modified to pass by reference
         * so that the simulation can be called several time and continue from last state
         * 
         */
        // this.bodies = bodies.clone();
        this.bodies = bodies;
        this.pq = new PriorityQueue<>();
        // (lzj)
        this.xmin = xmin;
        this.xmax = xmax;
        this.ymin = ymin;
        this.ymax = ymax;


        // set default double buffering 
        useDoubleBuffering(true);
    }

    // pushes the upcoming collision events to the priority queue if they occur within the specified time limit
    private void predict(Body a, double limit) {
        if (a == null) {
            return;
        }
        for (Body p : bodies) {
            // check possible collisions b/w a and p
            double dt = a.timeToHit(p);
            if (t + dt <= limit) {
                // if collision is possible within time limit, add to the pq
                pq.add(new Event(t + dt, a, p));
            }
        }
        double dtV = a.timeToHitVerticalWall(xmin, xmax);
        if (t + dtV <= limit) {
            pq.add(new Event(t + dtV, null, a));
        }
        double dtH = a.timeToHitHorizontalWall(ymin, ymax);
        if (t + dtH <= limit) {
            pq.add(new Event(t + dtH, a, null));
        }
    }

    // Handles the Redraw event by redrawing all the bodies with updated positions
    private void redraw(double limit) {
        StdDraw.clear();    // clear the canvas
        for (Body p : bodies) {
            p.draw();       // redraw each paricle
        }
        StdDraw.show();     // in case double buffering is used in StdDraw
        StdDraw.pause(20);  // freeze StdDraw for 20 ms so that frame may be observed

        // schedule redraw of frames based on Framerate frequency 
        if (t < limit) {
            pq.add(new Event(t + 1.0 / HZ, null, null));
        }
    }

    /**
     * A switch for a setting for the StdDraw library, may improve frame-rates
     * during simulation.
     *
     * @param yes if {@code true} double buffering is enabled (this is default),
     * otherwise it's disabled
     */
    public static void useDoubleBuffering(boolean yes) {
        if (yes) {
            StdDraw.enableDoubleBuffering();
        } else {
            StdDraw.disableDoubleBuffering();
        }
    }



    public void increment(double limit) {
        // initialize the PQ with collision events and redraw event

        /**
         * (lzj) attention here ! If you call "increment()" multiple times, you should set t=0 for new
         * simulation !!!!! 
         */
        pq = new PriorityQueue<>();
        t = 0;

        for (Body a : bodies) {
            predict(a, limit);
        }

        /// (lzj) attention ! modify here, t !
        pq.add(new Event(0, null, null));       // add redraw event


        /// (lzj) (test)
        StdOut.println("Start increment !");
        // for (int i = 0; i<4; ++i) {
        //     StdOut.printf("body[%d], (%.2f,%.2f,%.2f,%.2f) \n",i,bodies[i].rx,bodies[i].ry, bodies[i].vx,bodies[i].vy);
        // }
        StdOut.println();



        // the main event driven simulation loop
        while (!pq.isEmpty()) {

            // get impending event, drive the simulation, discard if invalids
            Event e = pq.remove();
            if (!e.isValid()) {
                continue;
            }

            // advance all bodies in time and bring them to time of current event
            for (Body p : bodies) {
                p.move(e.time - t);
            }
            t = e.time;         // advance the clock

            // update the body velocities
            Body a = e.a, b = e.b;
            if (a != null && b != null) {
                a.bounceOff(b);
            } else if (a != null) {
                a.bounceOffHorizontalWall();
            } else if (b != null) {
                b.bounceOffVerticalWall();
            } else {
                redraw(limit);
                continue;
            }

            predict(a, limit);      // add new events related to a 
            predict(b, limit);      // and b
        }




        /// (lzj) (test)
        // for (int i = 0; i<4; ++i) {
        //     StdOut.printf("body[%d], (%.2f,%.2f,%.2f,%.2f) \n",i,bodies[i].rx,bodies[i].ry, bodies[i].vx,bodies[i].vy);
        // }
        StdOut.println("Stop increment !");


        StdOut.println();
        StdOut.println();
        StdOut.println();
    }

    /**
     * Sets the number of redraw events per second. This value should be set in
     * proportion to the average speed of the paricles in the system. TOO HIGH
     * HZ with small speeds with result in extremely slow motion while TOO LOW
     * HZ with high speed might cause very fast and messy animation.
     * <p>
     * 0.5 is used as default value.
     *
     * @param HZ the new value of redraw frequency
     */
    public void setRedrawHZ(double HZ) {
        this.HZ = HZ;
    }

    /**
     * ************************************************************************
     * This class encapsulates the details associated with an event during
     * simulation. Event implicitly can be of three types based on whether a or
     * b are null or not. We use this strategy to avoid use of an Event type
     * classifying variable in the class.
     * <pre>
     *      - a and b both null:         redraw event
     *      - a null, b not null:        collision with vertical wall
     *      - a not null, b null:        collision with horizontal wall
     *      - a and b both not null      binary collision between a and b
     * </pre >
     **************************************************************************
     */
    private static class Event implements Comparable<Event> {

        public double time;             // time till collision event
        public Body a, b;           // the paricles which shall collide
        public int countA, countB;      // collision counts at Event creation

        // creates a new event scheduled at given time involving a and b
        public Event(double time, Body a, Body b) {
            this.time = time;
            this.a = a;
            this.b = b;
            if (a != null) {
                countA = a.count();
            } else {
                countA = -1;    // sentinel value
            }
            if (b != null) {
                countB = b.count();
            } else {
                countB = -1;
            }
        }

        @Override
        public int compareTo(Event that) {
            double dt = this.time - that.time;
            if (dt < 0) {
                return -1;
            } else if (dt > 0) {
                return +1;
            } else {
                return 0;
            }
        }

        // has any intervening event has occured since creation of this event
        public boolean isValid() {
            if (a != null && a.count() != countA) {
                return false;
            }
            if (b != null && b.count() != countB) {
                return false;
            }
            return true;
        }

    }

    //  for unit testing of the class
    public static void main(String[] args) {
        
        StdDraw.setCanvasSize(700, 700);
        
        // the array of bodies
        Body[] bodies;
        Body.useRadiusUpscaling(true);
        // create n random bodies
        if (args.length == 1) {
            int n = Integer.parseInt(args[0]);
            bodies = new Body[n];
            for (int i = 0; i < n; i++)
                bodies[i] = new Body();
        }

        // or read from standard input
        else {
            int n = StdIn.readInt();
            System.out.println(n + "bodies, Reading from STDIN...");
            bodies = new Body[n];
            for (int i = 0; i < n; i++) {
                double px     = StdIn.readDouble();
                double py     = StdIn.readDouble();
                double vx     = StdIn.readDouble();
                double vy     = StdIn.readDouble();
                double radius = StdIn.readDouble();
                double mass   = StdIn.readDouble();
                int r         = StdIn.readInt();
                int g         = StdIn.readInt();
                int b         = StdIn.readInt();
                Color color   = new Color(r, g, b);
                bodies[i] = new Body(px, py, vx, vy, mass, color, radius);
                System.out.println("Body added " + radius);
            }
        }

        IncrementEvent system = new IncrementEvent(bodies, 0.0, 1.0, 0.0, 1.0);
        system.setRedrawHZ(10);


        /// (lzj) TODO: try to modify it so that it can be called multiple time to start from last state
        system.increment(10000);

        // for(int i = 0; i<4; ++i){
        //     system.increment(5);            
        // }

        

        }

}


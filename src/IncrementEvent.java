import edu.princeton.cs.algs4.*;
import java.awt.Color;
import java.util.PriorityQueue;


public class  IncrementEvent {

    private double HZ = 0.5;    // redraw frequency as redraws per clock tick (in Simulator time)
    private PriorityQueue<Event> pq;    // the event priority queue
    private Body[] bodies;       // the array of bodies
    private double t = 0.0;                   // simulation clock time
    private final double xmin,xmax,ymin,ymax;

    /**
     * Constructs the simulator class with given array of bodies.
     * @param bodies the array of bodies
     * @param xmin the minimum x value of the global map
     * @param xmax the maximum x value of the global map
     * @param ymin the minimum y value of the global map 
     * @param ymax the maximum y value of the global map
     */
    public IncrementEvent(Body[] bodies, double xmin, double xmax, double ymin, double ymax) {
        // (lzj) the bodies should be modified to pass by reference
        this.bodies = bodies;
        this.pq = new PriorityQueue<>();
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
        StdDraw.clear(); 
        for (Body p : bodies) {
            p.draw();      
        }
        StdDraw.show();     
        StdDraw.pause(1);  

        if (t < limit) {
            pq.add(new Event(t + 1.0 / HZ, null, null));
        }
    }

    public static void useDoubleBuffering(boolean yes) {
        if (yes) {
            StdDraw.enableDoubleBuffering();
        } else {
            StdDraw.disableDoubleBuffering();
        }
    }

    public void increment(double limit) {

        // (lzj) attention here ! If you call "increment()" multiple times, you should set t=0
        pq = new PriorityQueue<>();
        t = 0.0;

        for (Body a : bodies) {
            a.count = 0;
            predict(a, limit);
        }

        pq.add(new Event(0.0, null, null));   


        // the main event driven simulation loop
        while (!pq.isEmpty()) {        

            // get impending event, drive the simulation, discard if invalids
            Event e = pq.remove();

            if (!e.isValid())
                continue;

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
            String gui_terminal = StdIn.readString();
            double map_radius = StdIn.readDouble();
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
        system.increment(10000);

        }
}


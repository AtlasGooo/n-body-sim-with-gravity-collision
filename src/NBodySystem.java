
import java.awt.Color;
import edu.princeton.cs.algs4.*;
import java.lang.Math;

    /**
     * @apiNote Dynamic dt and dynamic hz:
     * @apiNote dt = C1 * ( min r/v)
     * @apiNote HZ = 1 / ( C2 * dt )
     * @apiNote Recommended C1 = 0.3 or 0.8 , C2 = 0.8
     * @apiNote Note that this value can't be modified causually since it may cause mistakes
     */
public class NBodySystem {

    /**
     * Auxiliary function to dynamically modify dt according to bodies r/v
     */
    public static double dynamicDt(Body[] bodies, double C1){

        double min_rv_ratio = 0.2;
        double vmax = 0.0;

        for(Body b : bodies){
            vmax = Math.max(Math.abs(b.vx), Math.abs(b.vy) );
            if(vmax == 0.0)
                continue;
            double rv_ratio = b.radius / vmax;
            if(min_rv_ratio > rv_ratio){
                min_rv_ratio = rv_ratio; 
            }
        }
        return C1 * min_rv_ratio;
    }

    /**
     * Auxiliary function to dynamically modify re-draw hz according to dt
     */    
    public static double dynamicHz(double dt, double C2){
        // attention ! the constraint: 1/hz < dt !!!!
        if(C2 >= 1.0)
            C2 = 0.9;
        return 1.0/(C2*dt) ;
    }

    public static void main(String[] args) {
        
        StdDraw.setCanvasSize(700,700);

        String gui_terminal = StdIn.readString();
        double map_radius = StdIn.readDouble();      // map_radius of universe
        int N = StdIn.readInt();                 // number of particles

        // turn on animation mode and rescale coordinate system
        StdDraw.show(0);

        StdDraw.setXscale(0, +map_radius);
        StdDraw.setYscale(0, +map_radius);

        // read in and initialize bodies
        Body[] bodies = new Body[N];               // array of N bodies
        for (int i = 0; i < N; i++) {
            double px   = StdIn.readDouble();
            double py   = StdIn.readDouble();
            double vx   = StdIn.readDouble();
            double vy   = StdIn.readDouble();
            double radius = StdIn.readDouble();
            double mass = StdIn.readDouble();
            int red     = StdIn.readInt();
            int green   = StdIn.readInt();
            int blue    = StdIn.readInt();
            Color color = new Color(red, green, blue);

            bodies[i]   = new Body(px, py, vx, vy, mass, color, radius);
        }


        // You can't modify C1 and C2 casually since it may cause a bug ...
        // final double C1 = 0.3 or 0.8, C2 = 0.8;  
        final double C1 = 0.8, C2 = 0.8;
        double dt = 1.0; 
        double hz = 30.0;

        IncrementEvent increment_sys = new IncrementEvent(bodies, 0.0, map_radius, 0.0, map_radius);

        // set dt for first loop
        dt = dynamicDt(bodies, C1); 
        hz = dynamicHz(dt, C2);
        increment_sys.setRedrawHZ(hz);        

        // (lzj) (test)
        StdOut.printf("\nInitial : dt=%.7f, hz=%.7f \n\n",dt,hz);   
        
        for (double t = 0.0; true; t = t + dt) {

            Quad quad = new Quad(0.5*map_radius, 0.5*map_radius, map_radius * 2);
            BHTree tree = new BHTree(quad);

            // build the Barnes-Hut tree
            for (int i = 0; i < N; i++)
                if (bodies[i].in(quad))
                   tree.insert(bodies[i]);
        
            // update the forces, velocities, and accelerations
            for (int i = 0; i < N; i++) {
                bodies[i].resetForce();
                tree.updateForce(bodies[i]);
                bodies[i].updateVelocity(dt);
            }

            // after update velocity, use event base method to execute increment
            increment_sys.increment(dt);

            // dynamically change dt and hz for next loop
            dt = dynamicDt(bodies, C1);
            hz = dynamicHz(dt, C2);
            increment_sys.setRedrawHZ(hz);

            // (lzj) (test) 
            StdOut.printf("dt=%.3f, hz=%.3f \n",dt,hz);

        }

    }
}

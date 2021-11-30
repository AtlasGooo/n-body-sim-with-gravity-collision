/**
 * NBodyBH.java
 *
 * Reads in a universe of N bodies from stdin, and performs an
 * N-Body simulation in O(N log N) using the Barnes-Hut algorithm.
 *
 * Compilation:  javac NBodyBH.java
 * Execution:    java NBodyBH < inputs/[filename].txt
 * Dependencies: BHTree.java Body.java Quad.java StdDraw.java
 * Input files:  ./inputs/*.txt
 *
 * @author chindesaurus
 * @version 1.00
 */

import java.awt.Color;
// import java.util.Scanner;
import edu.princeton.cs.algs4.*;

import java.lang.Math;


public class NBodyBH {
    
    /**
     * Auxiliary function to dynamically modify dt according to bodies r/v
     */
    public static double dynamicDt(Body[] bodies, double C1, double default_dt){
        /**
         * dt = C1 * ( min r/v)
         * HZ = 1 / ( C2 * dt )
         * For p10.txt, initially use C1 = 1, C2 = 100/3 ~= 33.3
         */
        
        double min_rv_ratio = 1.0;
        for(Body b : bodies){
            double rv_ratio = b.radius / Math.max(Math.abs(b.vx), Math.abs(b.vy) );
            if(min_rv_ratio > rv_ratio){
                min_rv_ratio = rv_ratio; 
            }
        }
        
        double dt = Math.min(default_dt, C1*min_rv_ratio);
        return dt;
    }

    /**
     * Auxiliary function to dynamically modify re-draw hz according to dt
     */    
    public static double dynamicHz(double dt, double C2){
        return (double)1 / (C2*dt);
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
            

            // (lzj) (test)
            // mass *= 7E16;
            mass *= 7E18;

            if(i == 5){
                mass *= 10;
                radius *= 3;
            }


            bodies[i]   = new Body(px, py, vx, vy, mass, color, radius);
        }


        /**
         * dt = C1 * ( min r/v)
         * HZ = 1 / ( C2 * dt )
         * default C1 = 0.2, C2 = 3.33
         * 
         */
        final double C1 = 0.2, C2 = 5;
        final double default_dt = 0.01;
        final double default_hz = 30.0;
        double dt = default_dt; 
        double hz = default_hz;

        IncrementEvent increment_sys = new IncrementEvent(bodies, 0.0, map_radius, 0.0, map_radius);
        increment_sys.setRedrawHZ(hz);
        


    

        

        for (double t = 0.0; true; t = t + dt) {

            
            Quad quad = new Quad(0, 0, map_radius * 2);
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

            dt = dynamicDt(bodies, C1, default_dt);
            hz = dynamicHz(dt, C2);

            StdOut.printf("dt=%.3f, hz=%.3f \n",dt,hz);

            increment_sys.setRedrawHZ(hz);



        }

    }
}

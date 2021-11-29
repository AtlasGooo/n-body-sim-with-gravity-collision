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
import java.util.Scanner;
import edu.princeton.cs.algs4.*;

import java.lang.Math;


public class NBodyBH {

    public static void main(String[] args) {
        
        StdDraw.setCanvasSize(700,700);

        // for reading from stdin
        Scanner console = new Scanner(System.in);
        
        /// (lzj)
        final double dt = 0.005;                     // time quantum


        int N = console.nextInt();                 // number of particles
        // double map_radius = console.nextDouble();      // map_radius of universe
        double map_radius = 1.0;

        /// (lzj) for testing
        // if(N > 3)
        // N = 3;
        



        // turn on animation mode and rescale coordinate system
        StdDraw.show(0);

        // (lzj) TODO: in the Body's time to hit horizontal and verticle, the canvas default height and width is 1.0
        // StdDraw.setXscale(-map_radius, +map_radius);
        // StdDraw.setYscale(-map_radius, +map_radius);
        StdDraw.setXscale(0, +map_radius);
        StdDraw.setYscale(0, +map_radius);



        // read in and initialize bodies
        Body[] bodies = new Body[N];               // array of N bodies
        for (int i = 0; i < N; i++) {
            double px   = console.nextDouble();
            double py   = console.nextDouble();
            double vx   = console.nextDouble();
            double vy   = console.nextDouble();
            double radius = console.nextDouble();
            double mass = console.nextDouble();
            int red     = console.nextInt();
            int green   = console.nextInt();
            int blue    = console.nextInt();
            Color color = new Color(red, green, blue);
            

            // (lzj)
            // double rand_radius = 1E04 *( 0.5 + 2*Math.random() );
            mass *= 1E16;

            if(i == 5){
                mass *= 10;
                radius *= 3;
            }

            bodies[i]   = new Body(px, py, vx, vy, mass, color, radius);
        }



        // (lzj)
        IncrementEvent increment_sys = new IncrementEvent(bodies);
        increment_sys.setRedrawHZ(10);
        


        // for(double t = 0.0; true; t = t+dt){
        //     increment_sys.increment(dt);
        // }


        for (double t = 0.0; true; t = t + dt) {

            Quad quad = new Quad(0, 0, map_radius * 2);
            BHTree tree = new BHTree(quad);

            // build the Barnes-Hut tree
            for (int i = 0; i < N; i++)
                if (bodies[i].in(quad))
                   tree.insert(bodies[i]);
        
            // update the forces, positions, velocities, and accelerations
            for (int i = 0; i < N; i++) {
                bodies[i].resetForce();
                tree.updateForce(bodies[i]);

                bodies[i].updateVelocity(dt);

                // (lzj) TODO: raplace move() with event base simulation in the future
                // bodies[i].move(dt);

            }

            increment_sys.increment(dt);




            // (lzj) change background color from black to white)            
            // StdDraw.clear(StdDraw.BLACK);
            // StdDraw.clear();            

            // for (int i = 0; i < N; i++)
            //     bodies[i].draw();
            // StdDraw.show(10);
        }






    }
}

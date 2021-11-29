/**
 * Body.java
 *
 * Represents a Body (a point mass) and its position, 
 * velocity, mass, color, and the net force acting upon it.
 *
 * @author chindesaurus
 * @version 1.00
 */
import java.awt.Color;
import edu.princeton.cs.algs4.*;

public class Body {

    // gravitational constant
    private static final double G = 6.67e-11;

    private double rx, ry;       // position
    private double vx, vy;       // velocity
    private double fx, fy;       // force
    private double mass;         // mass
    private Color color;         // color


    // (lzj) make changes on "Body" to adapt "CollisionSystem"
    private static final double INFINITY = Double.POSITIVE_INFINITY;
    private static Color defaultColor = Color.BLACK;
    private static boolean useRadiusUpscaling = false;

    private int count;
    private final double radius;

    /**
     * Constructor: creates and initializes a new Body.
     *
     * @param rx    the x-position of this new body
     * @param ry    the y-position of this new body
     * @param vx    the x-velocity of this new body
     * @param vy    the y-velocity of this new body
     * @param mass  the mass of this new body
     * @param color the color of this new body (RGB)
     */
    public Body(double rx, double ry, double vx, double vy, double mass, Color color, double radius) {
        // (lzj) 
        this.count = 0;
        this.radius = radius;

        this.rx    = rx;
        this.ry    = ry;
        this.vx    = vx;
        this.vy    = vy;
        this.mass  = mass;
        this.color = color;
    }

    /**
     * Updates the velocity and position of the invoking Body
     * using leapfrom method, with timestep dt.
     * 
     * (lzj) divide "update()" into updateVelocity() and move()
     *
     * @param dt the timestep for this simulation
     */
    public void updateVelocity(double dt) {
        vx += dt * fx / mass;
        vy += dt * fy / mass;
        // rx += dt * vx;
        // ry += dt * vy;
    }
    public void move(double dt){
        rx += dt * vx;
        ry += dt * vy;        
    }

    /**
     * (lzj) Returns the total number of collisions involving this particle.
     * @return 
     */
    public int count(){
        return count;
    }


    /**
     * Returns the Euclidean distance between the invoking Body and b.
     *
     * @param b the body from which to determine the distance
     * @return  the distance between this and Body b
     */
    public double distanceTo(Body b) {
        double dx = rx - b.rx;
        double dy = ry - b.ry;
        return Math.sqrt(dx*dx + dy*dy);
    }

    /**
     * Resets the force (both x- and y-components) of the invoking Body to 0.
     * 
     * (lzj) TODO: modify reset force to reset() ?
     */
    public void resetForce() {
        fx = 0.0;
        fy = 0.0;
    }

    /** 
     * Computes the net force acting between the invoking body and b, and
     * adds this to the net force acting on the invoking Body.
     *
     * @param b the body whose net force on this body to calculate
     */
    public void addForce(Body b) {
        Body a = this;
        double EPS = 3E4;      // softening parameter
        double dx = b.rx - a.rx;
        double dy = b.ry - a.ry;
        double dist = Math.sqrt(dx*dx + dy*dy);
        double F = (G * a.mass * b.mass) / (dist*dist + EPS*EPS);
        a.fx += F * dx / dist;
        a.fy += F * dy / dist;
    }

    /**
     * Draws the invoking Body. 
     * (lzj) modify draw point to draw circle
     */
    public void draw() {
        StdDraw.setPenColor(color);
        // StdDraw.point(rx, ry);
        StdDraw.filledCircle(rx, ry, radius);
    }

    /**
     * Returns a string representation of this body formatted nicely.
     *
     * @return a formatted string containing this body's x- and y- positions,
     *         velocities, and mass
     */
    public String toString() {
        return String.format("%10.3E %10.3E %10.3E %10.3E %10.3E", rx, ry, vx, vy, mass);
    }

    /**
     * Returns true if the body is in quadrant q, else false.
     *
     * @param q the Quad to check
     * @return  true iff body is in Quad q, else false
     */
    public boolean in(Quad q) {
        return q.contains(this.rx, this.ry); 
    }

    /** 
     * Returns a new Body object that represents the center-of-mass
     * of the invoking body and b.
     *
     * @param b the body to aggregate with this Body
     * @return  a Body object representing an aggregate of this 
     *          and b, having this and b's center of gravity and
     *          combined mass
     */
    public Body plus(Body b) {
        Body a = this;

        double m = a.mass + b.mass;
        double x = (a.rx * a.mass + b.rx * b.mass) / m;
        double y = (a.ry * a.mass + b.ry * b.mass) / m;

        /// (lzj) TODO: radius = -1 means aggregate body !!!!! check if exist mistakes !!!!!
        return new Body(x, y, a.vx, b.vx, m, a.color, -1);
    }
}

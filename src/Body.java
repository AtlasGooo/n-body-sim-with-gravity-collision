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
    // (lzj) temporary change some properties to public for printing info
    public double rx, ry;       // position
    public double vx, vy;       // velocity
    private double fx, fy;       // force
    private double mass;         // mass
    private Color color;         // color




    // (lzj) make changes on "Body" to adapt "CollisionSystem"
    private static final double INFINITY = Double.POSITIVE_INFINITY;
    private static Color defaultColor = Color.BLACK;
    private static boolean useRadiusUpscaling = false;

    private int count;
    private final double radius;


    // (lzj)
    public Body() {
        this.count = 0;

        this.rx = StdRandom.uniform(0.5, 0.95);
        this.ry = StdRandom.uniform(0.5, 0.95);
        this.vx = StdRandom.uniform(-0.5, 0.5);
        this.vy = StdRandom.uniform(-0.5, 0.5);
        this.radius = 0.01;
        this.mass = 0.5;
        this.color = defaultColor;
    }

    /**
     * Constructor: creates and initializes a new Body.
     *
     * @param rx    the x-position of this new body
     * @param ry    the y-position of this new body
     * @param vx    the x-velocity of this new body
     * @param vy    the y-velocity of this new body
     * @param mass  the mass of this new body
     * @param color the color of this new body (RGB)
     * @param radius the radius of this body
     */
    public Body(double rx, double ry, double vx, double vy, double mass, Color color, double radius) {
        // (lzj) add
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

    // (lzj)
    public double timeToHit(Body that){
        if (this == that) {
            return INFINITY;
        }
        double dx = that.rx - this.rx;          // x-axis displacement
        double dy = that.ry - this.ry;          // y-axis displacement
        double dvx = that.vx - this.vx;         // x-axis relative velocity
        double dvy = that.vy - this.vy;         // y-axis relative velocity

        // dot product of vector dr and vector dv; predicts the existence of finite time to collide
        double dvdr = dvx * dx + dvy * dy;
        if (dvdr > 0) {
            return INFINITY;        // I know this physics is weird
        }

        double dvdv = dvx * dvx + dvy * dvy;    // magnitude of dv
        if (dvdv == 0) {
            return INFINITY;        // Means relative velocity is zero
        }

        double drdr = dx * dx + dy * dy;    // magnitude of dr vector

        double sigma = that.radius + this.radius;       // sum of radii of colliding particles

        // this discriminant comes from solution of a quadratic equation, see some Physics dude.
        double discriminant = dvdr * dvdr - dvdv * (drdr - sigma * sigma);
        if (drdr < sigma * sigma) {
            System.out.println("Particles overlap !!! UNEXPECTED behaviour expected :p");
        }
        if (discriminant < 0) {
            return INFINITY;    // there are no solutions to equation or the time to collide 
        }
        return -(dvdr + Math.sqrt(discriminant)) / dvdv;
        // we ignore the other solution (Why ? because there can be two collisons in Mathematics; think about it, try it)
    }

    // (lzj)
    public double timeToHitHorizontalWall(double ymin, double ymax) {
        if (vy < 0) {
            return (radius - ry - ymin) / vy;
        } else if (vy > 0) {
            return (ymax - ry - radius) / vy;
        } else {
            return INFINITY;
        }
    }  

    // (lzj)
    public double timeToHitVerticalWall(double xmin, double xmax) {
        if (vx < 0) {
            return (radius - rx - xmin) / vx;
        } else if (vx > 0) {
            return (xmax - rx - radius) / vx;
        } else {
            return INFINITY;
        }
    }

    // (lzj)
    public void bounceOff(Body that) {
        double dx = that.rx - this.rx;
        double dy = that.ry - this.ry;
        double dvx = that.vx - this.vx;
        double dvy = that.vy - this.vy;
        // dot product of dv vector and dr vector
        double dvdr = dvx * dx + dvy * dy;
        // sum of the radii of this and that particle
        double sigma = this.radius + that.radius;

        // total magnitude of impulse exchanged on collision
        double j = 2 * dvdr * this.mass * that.mass / ((this.mass + that.mass) * sigma);

        // x and y components of the Impulse 
        double jx = j * dx / sigma;
        double jy = j * dy / sigma;

        // update velocity according to momentum change given by impulse
        this.vx += jx / this.mass;
        this.vy += jy / this.mass;
        that.vx -= jx / that.mass;
        that.vy -= jy / that.mass;

        // update collision counts
        this.count++;
        that.count++;
    }

    // (lzj)
    public void bounceOffVerticalWall() {
        this.vx = -this.vx;
        count++;
    }

    // (lzj)
    public void bounceOffHorizontalWall() {
        this.vy = -this.vy;
        count++;
    }

    public double kineticEnergy() {
        // elementary formula 1/2 * m * v * v 
        return (mass * (vx * vx + vy * vy)) / 2;
    }

    public void setDefaultColor(Color newColor) {
        Body.defaultColor = newColor;
    }

    public static void useRadiusUpscaling(boolean yes) {
        if (yes) {
            useRadiusUpscaling = true;
        } else {
            useRadiusUpscaling = false;
        }
    }

    public static boolean radiusUpscalingEnabled() {
        return useRadiusUpscaling;
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

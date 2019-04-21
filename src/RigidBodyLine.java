/*
 * File added by Nathan MacLeod 2019
 */
import java.awt.Color;
import java.awt.Graphics;
/**
 *
 * @author macle
 */
public class RigidBodyLine extends Line {
    private boolean normalClockwiseP1; //Indicates if the normal vector goes cockwise or counterclock wise around p1
    private RigidBody body;
    
    public RigidBodyLine(Point p1, Point p2, RigidBody body) {
        super(p1, p2);
        this.body = body;
    }
    
    public RigidBody getBody() {
        return body;
    }
    
    private double[] createVectorComponentsOfMagnitude(double magnitude, double slope, boolean xComponentPositive) {
        int xSign = -1;
        if(xComponentPositive)
            xSign = 1;
        if(slope >= Double.MAX_VALUE || slope <= -Double.MAX_VALUE) {
           if(slope * xSign > 0)
               return new double[] {0, 1};
           return new double[] {0, -1};
        }
        double xCompMagnitude = xSign * Math.sqrt(magnitude/(1 + Math.pow(slope, 2)));
        double yCompMagnitude = slope * xCompMagnitude;
        return new double[] {xCompMagnitude, yCompMagnitude};
    }
    
    public Vector getNormalUnitVector() {
        //Keep in mind that the unit circle is upside down due to origin being in top left corner
        double relativeY = p2.y - p1.y;
        if(relativeY == 0) {
            if((p2.x > p1.x && normalClockwiseP1) || (p2.x < p1.x && !normalClockwiseP1))
                return new Vector(0, 1);
            return new Vector(0, -1);
        }
        boolean xDirectionPositive = false;
        if((relativeY > 0 && !normalClockwiseP1) || (relativeY < 0 && normalClockwiseP1))
            xDirectionPositive = true;
        double normalSlope = -1.0/getSlope();
        double[] components = createVectorComponentsOfMagnitude(1, normalSlope, xDirectionPositive);
        return new Vector(components[0], components[1]);
    }
    
    public void findNormalDirection(RigidBody body) {
        Point p;
        double pDistFromLine = 0.01;
        boolean pInsideBody;
        
        double relativeX = p2.x - p1.x;
        double relativeY = p2.y - p1.y;
        
        
        if(relativeY == 0) {
            
            p = new Point((p2.x + p1.x)/2.0, (p1.y + p2.y)/2.0 + pDistFromLine);
            pInsideBody = body.pointInsideBody(p);
            if((pInsideBody && relativeX < 0) || !pInsideBody && relativeX > 0)
                normalClockwiseP1 = true;
            else {
                normalClockwiseP1 = false;
            }
            return;
        }
        double[] p1ToP = createVectorComponentsOfMagnitude(pDistFromLine, -1.0/getSlope(), relativeY < 0);
        p = new Point((p2.x + p1.x)/2.0 + p1ToP[0], (p1.y + p2.y)/2.0 + p1ToP[1]);
        pInsideBody = body.pointInsideBody(p);
        if(pInsideBody)
            normalClockwiseP1 = false;
        else
            normalClockwiseP1 = true;
    }
}

/*
 * File added by Nathan MacLeod 2019
 */

/**
 *
 * @author Nathan
 */
public class Vector {
    private double xComp;
    private double yComp;
    
    public Vector(double xComp, double yComp) {
        this.xComp = xComp;
        this.yComp = yComp;
    }
    
    public double getMagnitude() {
        return Math.sqrt((xComp * xComp) + (yComp * yComp));
    }
    
    public Vector getUnitVector() {
        double mag = getMagnitude();
        return new Vector(xComp/mag, yComp/mag);
    }
    
    public double getSquaredMagnitude() {
        return (xComp * xComp) + (yComp * yComp);
    }
    
    public double getDirection() {
        double angle = Math.atan(yComp/xComp);
        if(yComp < 0)
            angle += Math.PI * 2;
        if(xComp < 0 && yComp < 0)
            angle += Math.PI;
        else if(xComp < 0 && yComp >= 0)
            angle += Math.PI;
        while(angle > Math.PI * 2)
            angle -= Math.PI * 2;
        while(angle < 0)
            angle += Math.PI * 2;
        return angle;
    }
    
    public double getXComp() {
        return xComp;
    }
    
    public double getYComp() {
        return yComp;
    }
    
    public Vector add(Vector v) {
        return new Vector(xComp + v.getXComp(), yComp + v.getYComp());
    }
    
    public Vector subtract(Vector v) {
        return add(v.multiplyByScalar(-1));
    }
    
    public Vector perpendicularize() {
        double xPart = Math.abs(yComp);
        double yPart = Math.abs(xComp);
        if(yComp > 0) 
            xPart *= -1;
        if(xComp < 0)
            yPart *= -1;
        return new Vector(xPart, yPart);
    }
    
    public double dotProduct(Vector v) {
        return (v.xComp * xComp) +  (v.yComp * yComp);
    }    
    
    public Vector multiplyByScalar(double scal) {
        return new Vector(xComp * scal, yComp * scal);
    }
    
    public String toString() {
        return "X component: " + xComp + ", yComponenet: " + yComp + ", Magnitude: " + getMagnitude() + ", Direction: " + getDirection() * 180.0/Math.PI + " degrees";
    }
}

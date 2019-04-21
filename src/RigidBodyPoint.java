/*
 * File added by Nathan MacLeod 2019
 */

/**
 *
 * @author macle
 */
public class RigidBodyPoint extends Point {
    private RigidBodyLine[] lines;
    private RigidBody body;
    
    public RigidBodyPoint(double x, double y) {
        super(x, y);
        lines = new RigidBodyLine[2];
        this.body = body;
    } 
    
    public void addLine(RigidBodyLine l) {
        if(lines[0] != null)
            lines[1] = l;
        else
            lines[0] = l;
    }
    
    public void setBody(RigidBody b) {
        body = b;
    }
    
    public RigidBody getBody() {
        return body;
    }
    
    public RigidBodyLine[] getLines() {
        return lines;
    }
    
}

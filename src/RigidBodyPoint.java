/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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

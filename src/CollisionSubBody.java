/*
 *File added by Nathan MacLeod 2019
 */
import java.util.ArrayList;
import java.awt.Graphics;
import java.awt.Color;
/**
 *
 * @author Nathan
 */
public class CollisionSubBody {
    private ArrayList<RigidBodyPoint> nodes;
    private Line[] subBodyLines;
    
    public CollisionSubBody(ArrayList<RigidBodyPoint> points) {
        nodes = points;
    }
    
    public ArrayList<RigidBodyPoint> getNodes() {
        return nodes;
    }
    
    public Line[] getLines() {
        return subBodyLines;
    }
    
    private void addAfterP1(RigidBodyPoint p, RigidBodyPoint p1) {
        for(int i = 0; i < nodes.size(); i++) {
            if(nodes.get(i).equals(p1)) {
                nodes.add(i + 1, p);
                break;
            }
        }
    }
    
    private void findSubBodyLines() {
        subBodyLines = new Line[nodes.size()];
        for(int i = 0; i < nodes.size(); i++) {
            Point p1 = nodes.get(i);
            Point p2;
            if(i == nodes.size() - 1)
                p2 = nodes.get(0);
            else
                p2 = nodes.get(i + 1);
            subBodyLines[i] = new Line(p1, p2);
        }
    }
    
    public boolean pointInsideBody(Point p) {
        //Creates a horizontel line centered around P and checks how it intersects with 
        //the lines in the body, if the number of intersects to the left and right of p are odd numbers p is inside the body
        Line l = new Line(new Point(-Integer.MAX_VALUE, p.y), new Point(Integer.MAX_VALUE, p.y));
        int intersectsLeft = 0;
        int intersectsRight = 0;
        for(int i = 0; i < subBodyLines.length; i++) {
            Point intersect = subBodyLines[i].getIntersection(l);
            if(intersect == null)
                continue;
            
            int acc = 10000;
            if((int)(intersect.x * acc) == (int)(p.x * acc))
                return true;
            else if(intersect.x < p.x)
                intersectsLeft++;
            else if(intersect.x > p.x)
                intersectsRight++;
        }
        return (intersectsLeft % 2) == 1 && (intersectsRight % 2) == 1;
    }
    
    public boolean findIfConvex() {
        if(nodes.size() < 4)
            return true;
        for(int i = 0; i < nodes.size(); i++) {
            Point p1 = nodes.get(i);
            int p2Index = (i + 2) % nodes.size();
            Point p2 = nodes.get(p2Index);
            boolean notConcave = pointInsideBody(new Line(p1, p2).getMidPoint());
            if(!notConcave)
                return false;
        }
        return true;
    }
    
    public static ArrayList<CollisionSubBody> breakIntoSubBodies(ArrayList<RigidBodyPoint> points) {
        ArrayList<CollisionSubBody> subBodies = new ArrayList<>();
        //Initializaton
        CollisionSubBody remainingBody = new CollisionSubBody(points);
        remainingBody.findSubBodyLines();
        PointNode start = new PointNode(points.get(0));
        //Assemble the pointnode data structure
        int size = points.size();
        PointNode current = start;
        for(int i = 1; i <= points.size(); i++) {
            if(i == points.size()) {
                current.next = start;
                start.previous = current;
            }
            else {
                current.next = new PointNode(points.get(i));
                current.next.previous = current;
                current = current.next;
            }
        }
        while(size > 3) {
            //Find valid start
            while(!isStartValid(start, remainingBody)) {
                start = start.next;
            }
            //intialize next body to add and p1 and p2
            CollisionSubBody nextSubBody = new CollisionSubBody(new ArrayList<RigidBodyPoint>());
            ArrayList<RigidBodyPoint> nextBodyPoints = nextSubBody.getNodes();
            PointNode p1 = start.next;
            PointNode p2 = start.previous;
            nextBodyPoints.add(p1.point);
            nextBodyPoints.add(p2.point);
            nextBodyPoints.add(start.point);
            
            //Loop, meat of the algorithim
            boolean subBodyFound = false;
            int pointsAdded = 0;
            boolean expandedRightLast = false;
            while(!subBodyFound) {
                //Find next point to expand to
                boolean expandedRight = false;
                PointNode nextP = null;
                
                while(true) {

                    if(expandedRightLast) {
                        nextP = p1.next;
                        if(nextP == p2) {
                            subBodyFound = true;
                            break;
                        }
                        expandedRight = testExpand(p2, nextP, remainingBody, nextSubBody);
                        if(expandedRight)
                            break;
                        nextP = p2.previous;
                        expandedRight = !testExpand(p1, nextP, remainingBody, nextSubBody);
                        if(!expandedRight)
                            break;
                        subBodyFound = true;
                        break;
                    }
                    else {
                        nextP = p2.previous;
                        if(nextP == p1) {
                            subBodyFound = true;
                            break;
                        }
                        expandedRight = !testExpand(p1, nextP, remainingBody, nextSubBody);
                        if(!expandedRight)
                            break;
                        nextP = p1.next;
                        expandedRight = testExpand(p2, nextP, remainingBody, nextSubBody);
                        if(expandedRight)
                            break;
                        subBodyFound = true;
                        break;
                    }
                }
                if(subBodyFound)
                    break;
                if(expandedRight) {
                    p1 = nextP;
                    expandedRightLast = true;
                }
                else {
                    p2 = nextP;
                    expandedRightLast = false;
                }
                pointsAdded++;
            }
            //cut off the new section
            nextSubBody.findSubBodyLines();
            subBodies.add(nextSubBody);

            PointNode p = p2.next;
            while(p != p1) {
                remainingBody.getNodes().remove(p.point);
                p = p.next;
            }
            remainingBody.findSubBodyLines();
            size -= pointsAdded + 1;
            p2.next = p1;
            p1.previous = p2;
            start = p1;
            
        }
        if(size == 3) {
            remainingBody.findSubBodyLines();
            subBodies.add(remainingBody);
        }
        return subBodies;
            
    }
    
    private static boolean testExpand(PointNode crossPoint, PointNode nextP, CollisionSubBody remainingBody, CollisionSubBody nextSubBody) {
        if(!lineIntersectsWithBody(remainingBody, new Line(crossPoint.point, nextP.point)) && remainingBody.pointInsideBody(new Line(crossPoint.point, nextP.point).getMidPoint())) {
            nextSubBody.addAfterP1(nextP.point, crossPoint.point);
            nextSubBody.findSubBodyLines();
            if(nextSubBody.findIfConvex()) {
                return true;
            }
            nextSubBody.getNodes().remove(nextP.point);
        }
        return false;
    }
    
    private static boolean isStartValid(PointNode startPoint, CollisionSubBody structure) {
        Point p1 = startPoint.next.point;
        Point p2 = startPoint.previous.point;
        
        boolean p1p2Valid = structure.pointInsideBody(new Line(p1, p2).getMidPoint()) && !lineIntersectsWithBody(structure, new Line(p1, p2));
        
        return p1p2Valid;
    }
    
    private static boolean lineIntersectsWithBody(CollisionSubBody body, Line line) {
        for(Line l : body.getLines()) {
            if(l.p1.equals(line.p1) || l.p2.equals(line.p1) || l.p1.equals(line.p2) || l.p2.equals(line.p2))
                continue;
            else if(line.getIntersection(l) != null)
                return true;
        }
        return false;
    }
    
    private static class PointNode {
        RigidBodyPoint point;
        PointNode next;
        PointNode previous;
        
        public PointNode(RigidBodyPoint p) {
            point = p;
        }
    }
    
    public void draw(Graphics g) {
       for(Line l : subBodyLines) {
            l.draw(g, Color.white);
        } 
    }
    
}

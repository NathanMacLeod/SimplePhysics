/*
 * File added by Nathan MacLeod 2019
 */
import javax.swing.JFrame;
import javax.swing.JPanel;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.awt.Color;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.Scanner;
/**
 *
 * @author Nathan
 */
public class PhysicsPanel extends JPanel implements Runnable {
    private ArrayList<RigidBody> bodies;
    private ArrayList<RigidBody> addBodyQueue;
    private ArrayList<RigidBody> deleteQueue;
    private boolean running;
    private Thread runner;
    private Vector gravity;
    private int currentIDIndex;
    private double timeStep;
    private double timeQueue;
    private double maxTimeQueue;
    private MouseTool mouseTool = new ForceTool();
    private double timeScale = 1;
    private boolean randomColor = true;
    private Color bodyColor = null;
    private boolean drawCOM = false;
    private boolean createFixedBody = false;
    private int nSides = 3;
    private double friction = 0.5;
    private double resistution = 0.3;
    private double density = 50;
    
    public void start() {
        running = true;
        runner = new Thread(this);
        runner.start();
    }       
    
    public void run() {
        currentIDIndex = 0;
        maxTimeQueue = 0.033;
        timeQueue = 0;
        timeStep = 0.003;
        bodies = new ArrayList<>();
        addBodyQueue = new ArrayList<>();
        deleteQueue = new ArrayList<>();
        gravity = new Vector(0, 300);
        bodies.add(new RigidRectangle(200, 800, 600, 30, new Vector(0, 0), 0, 100, 0.3, 0.5, true, giveID(), 0));
        bodies.add(new RigidBody(new RigidBodyPoint[] {new RigidBodyPoint(300, 400), new RigidBodyPoint(350, 314), new RigidBodyPoint(400, 400)}, new Vector(0, 0), 0, 100, 0.3, 0.5, false, giveID(), Color.BLUE, true));
        bodies.add(new RigidRectangle(500, 300, 200, 200, new Vector(0, 0), 0, 100, 0.3, 0.5, false, giveID(), 0));
        bodies.add(new RigidRectangle(525, 50, 150, 150, new Vector(0, 0), 0.1, 100, 0.3, 0.5, false, giveID(), 0));
        long currentTime;
        long pastTime = System.nanoTime();
        double timePassed = 0;
        while(running) {
            while(addBodyQueue.size() > 0)
                bodies.add(addBodyQueue.remove(0));
            while(deleteQueue.size() > 0)
                bodies.remove(deleteQueue.remove(0));
            currentTime = System.nanoTime();
            timePassed = (currentTime - pastTime)/Math.pow(10, 9);
            pastTime = currentTime;
            if(timeScale <= 1)
                timeQueue += timePassed;
            else
                timeQueue += timePassed * timeScale;
            if(timeQueue >= maxTimeQueue)
                timeQueue = maxTimeQueue;
            
            while(timeQueue >= timeStep) {
                if(timeScale <= 1)
                    physicsUpdate(timeStep * timeScale);
                else
                    physicsUpdate(timeStep);
                timeQueue -= timeStep;
                
            }
            render();
        }
    }
    
    private int giveID() {
        int id = currentIDIndex;
        currentIDIndex++;
        return id;
    }
    
    private void removeOutOfBoundsBodies() {
        for(int i = 0; i < bodies.size(); i++) {
            RigidBody b = bodies.get(i);
            double[] bounds = b.getMaxAndMinVals();
            if(!(mouseTool instanceof BodyManipulationTool && ((BodyManipulationTool)mouseTool).bodySelected(b)) && (bounds[2] < 0 || bounds[0] > getWidth() || bounds[1] > getHeight())) {
                bodies.remove(i);
                i--;
            }
        }
    }
    
    private void render() {
        BufferedImage img = new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_INT_ARGB);
        Graphics g = img.getGraphics();
        g.setColor(Color.black);
        g.fillRect(0, 0, getWidth(), getHeight());
        for(RigidBody b : bodies) {
            b.draw(g);
        }
        g.setColor(Color.white);
        mouseTool.drawTool(g);
        Graphics panelGraphics = getGraphics();
        panelGraphics.drawImage(img, 0, 0, null);
    }
    
    private void physicsUpdate(double dt) {
        if(mouseTool instanceof BodyManipulationTool)
            ((BodyManipulationTool)mouseTool).interactWithBodyBeforeUpdate(dt);
        for(RigidBody b : bodies) {
            if(!b.getAtRest())
                b.moveForwardOrBackInTime(dt, gravity);
            b.clearCollidedWithList();
        }         
        for(int i = 0; i < bodies.size(); i++) {
            RigidBody b = bodies.get(i);
            if(b.getFixed() || b.getAtRest()) 
                continue;
            
            for(int j = 0; j < bodies.size(); j++) {
                if(j == i)
                    continue;
                RigidBody b2 = bodies.get(j);
                if(b.collidedWith(b2.getID()))
                    continue;
                if(b.bodiesColliding(b2)) {
                    if(collideBodies(b, b2, dt)) {
                        if(b.tryingToRest() || b2.tryingToRest()) {
                            b.addRestBuddy(b2);
                            b2.addRestBuddy(b);
                        }
                        if(b2.getAtRest()) {
                            b2.breakRest(false);
                        }
                        b.addToCollidedWithList(b2.getID());
                        b2.addToCollidedWithList(b.getID());
                    }
                }
            }
        }
        for(RigidBody b : bodies) {
            b.determineIfAtRest(dt);
        }
        removeOutOfBoundsBodies();
        if(mouseTool instanceof BodyManipulationTool)
            ((BodyManipulationTool)mouseTool).interactWithBodyAfterUpdate();
    }
    
    private void pushBodiesApart(Object[] penInfo) {
        if(penInfo == null)
            return;

        RigidBodyPoint collidingPoint = (RigidBodyPoint) penInfo[0];
        RigidBodyLine collidingLine = (RigidBodyLine) penInfo[1];
        
        RigidBody lineBody = collidingLine.getBody();
        RigidBody pointBody = collidingPoint.getBody();
        Vector direct = (Vector)penInfo[3];
        double distScaleFactor = 1;
        double dist = (double)penInfo[2];
        double pushDist = dist * distScaleFactor;
        double lineBodyPushDist = pushDist/2.0;
        double pointBodyPushDist = pushDist - lineBodyPushDist;
        if(lineBody.getFixed()) {
            pointBodyPushDist = pushDist;
            lineBodyPushDist = 0;
        }
        else if(pointBody.getFixed()) {
            pointBodyPushDist = 0;
            lineBodyPushDist = pushDist;
        }
        pointBody.translate(direct.multiplyByScalar(pointBodyPushDist));
        lineBody.translate(direct.multiplyByScalar(-lineBodyPushDist));
    }
    
    private boolean collideBodies(RigidBody b1, RigidBody b2, double dt) {
        //Resolve a collision between two colliding bodies
        b1.moveForwardOrBackInTime(-dt, gravity);
        b2.moveForwardOrBackInTime(-dt, gravity);

        boolean alreadyInside = (b1.bodiesColliding(b2))? true : false;
        
        double maxSlackDist = 0.01;
        boolean outside = true;
        int count = 0;
        
        if(alreadyInside) {
            b1.moveForwardOrBackInTime(dt, gravity);
            b2.moveForwardOrBackInTime(dt, gravity);
        }
        
        Object[] penInfo = getPenInfoForCollision(b1, b2);
        
        while(!alreadyInside && (penInfo == null || ((double) penInfo[2] > maxSlackDist) && !outside)) {
            dt /= 2;
            count++;
            if(count == 80) {   
                break;
            }
            if(outside) {
                b1.moveForwardOrBackInTime(dt, gravity);
                b2.moveForwardOrBackInTime(dt, gravity);
            }
            else {
                b1.moveForwardOrBackInTime(-dt, gravity);
                b2.moveForwardOrBackInTime(-dt, gravity);
            }        
            Object[] newInfo = getPenInfoForCollision(b1, b2);
            if(newInfo == null) {
                outside = true;               
            }
            else {
                outside = false;
                penInfo = newInfo;
            }
        }
        if(penInfo == null) {
            penInfo = getPenInfoForPush(b1, b2);
            if(penInfo != null) {
                penInfo[2] = (double)penInfo[2] + 0.1;
                pushBodiesApart(penInfo);
            }
            return true;
        }
        RigidBodyPoint collidingPoint = (RigidBodyPoint) penInfo[0];
        RigidBodyLine collidingLine = (RigidBodyLine) penInfo[1];
        
        RigidBody lineBody = collidingLine.getBody();
        RigidBody pointBody = collidingPoint.getBody();
        
        Line pointsCollidingLine = getCollidingLineIfExists(collidingPoint, collidingLine);
        boolean lineCollision = false;
        if(pointsCollidingLine != null) {
            lineCollision = true;
            collidingPoint = getMidpointOfLineOverlap(collidingLine, pointsCollidingLine);
        }
       
        double resistution = (b1.getResistution() + b2.getResistution())/2.0;
        
        
        Vector normalVector = collidingLine.getNormalUnitVector();
        Vector Vap = lineBody.getVelocity().add(lineBody.getLinearVelocityOfPointDueToAngularVelocity(collidingPoint));
        Vector Vbp = pointBody.getVelocity().add(pointBody.getLinearVelocityOfPointDueToAngularVelocity(collidingPoint));
        Vector Vab = Vap.subtract(Vbp);
        
        double numerator = (1 + resistution) * Math.abs(Vab.dotProduct(normalVector));
        
        double denomenator = 
(b1.getInverseMass() + b2.getInverseMass()) + 
(Math.pow(normalVector.dotProduct(b1.getPerpendicularizedVectorToPoint(collidingPoint)), 2) * b1.getInverseMommentOfInertia())
+ (Math.pow(normalVector.dotProduct(b2.getPerpendicularizedVectorToPoint(collidingPoint)), 2) * b2.getInverseMommentOfInertia());
        
        double j = numerator/denomenator;
        
        double frictionJ;
        Vector t = normalVector.perpendicularize();
        double frictionNumerator = resistution * Vab.dotProduct(t);
        double frictionDenomenator = 
((b1.getInverseMass() + b2.getInverseMass()) + 
(Math.pow(t.dotProduct(b1.getPerpendicularizedVectorToPoint(collidingPoint)), 2) * b1.getInverseMommentOfInertia())
+ (Math.pow(t.dotProduct(b2.getPerpendicularizedVectorToPoint(collidingPoint)), 2) * b2.getInverseMommentOfInertia()));
        
        frictionJ = frictionNumerator/frictionDenomenator;
        double frictionCoefficent = (lineCollision)? (lineBody.getFrictionCoefficent() + pointBody.getFrictionCoefficent())/2.0 : lineBody.getFrictionCoefficent();
        if(Math.abs(frictionJ) > j * frictionCoefficent) {
            if(frictionJ < 0)
                frictionJ = -j * frictionCoefficent;
            else
                frictionJ = j * frictionCoefficent;
        }
        
        pointBody.applyImpulseOnPoint(collidingPoint, t.multiplyByScalar(frictionJ));
        lineBody.applyImpulseOnPoint(collidingPoint, t.multiplyByScalar(-frictionJ));
        pointBody.applyImpulseOnPoint(collidingPoint, normalVector.multiplyByScalar(j));
        lineBody.applyImpulseOnPoint(collidingPoint, normalVector.multiplyByScalar(-j));
        
        penInfo = getPenInfoForPush(b1, b2);
        if(penInfo == null)
            return true;
        penInfo[2] = (double)penInfo[2] + 0;
        pushBodiesApart(penInfo);
        
        return true;
    }
    
    private RigidBodyPoint getMidpointOfLineOverlap(Line l1, Line l2) {
        double smallX = l1.getSmallestX();
        if(l2.getSmallestX() > smallX)
            smallX = l2.getSmallestX();
        double smallY = l1.getSmallestY();
        if(l2.getSmallestY() > smallY)
            smallY = l2.getSmallestY();
        double bigX = l1.getLargestX();
        if(l2.getLargestX() < bigX)
            bigX = l2.getLargestX();
        double bigY = l1.getLargestY();
        if(l2.getLargestY() < bigX)
            bigY = l2.getLargestY();
        
        return new RigidBodyPoint((bigX + smallX)/2.0, (bigY + smallY)/2.0);
    }
    
    private Line getCollidingLineIfExists(RigidBodyPoint p, Line l) {
        double radianDiffTolerance = 0.0005;
        Line[] pointsLines = p.getLines();
        Line possibleLine = null;
        for(Line collLine : pointsLines) {
            if(Math.abs(Math.tan(collLine.getSlope()) - Math.tan(l.getSlope())) < radianDiffTolerance)
                possibleLine = collLine;
        }
        if(possibleLine == null)
            return null;
        return possibleLine;
    }
    
    private Object[] decidePenInfo(Object[] pointInB1, Object[] pointInB2) {
        if(pointInB1 == null && pointInB2 == null)
            return null;
        
        if(pointInB1 == null) {
            return pointInB2;
        }
        if(pointInB2 == null || (double)pointInB1[2] > (double)pointInB2[2]) {
            return pointInB1;
        }
        
        return pointInB2;
    }
    
    private Object[] getPenInfoForCollision(RigidBody b1, RigidBody b2) {
        Object[] pointInB1 = b1.getPenetratingPoint(b2, false, gravity);
        Object[] pointInB2 = b2.getPenetratingPoint(b1, false, gravity);
        return decidePenInfo(pointInB1, pointInB2);
    }
    
    private Object[] getPenInfoForPush(RigidBody b1, RigidBody b2) {
        Object[] pointInB1 = b1.getPenetratingPoint(b2, true, gravity);
        Object[] pointInB2 = b2.getPenetratingPoint(b1, true, gravity);
        return decidePenInfo(pointInB1, pointInB2);
    }
    
    public void updateNSides(int n) {
        nSides = n;
    }
    
    public void updateFixed(boolean b) {
        createFixedBody = b;
    }
    
    public void updateDrawCOM(boolean b) {
        drawCOM = b;
    }
    
    public void updateDensity(Double d) {
        density = d;
    }
    
    public void updateFriction(Double d) {
        friction = d;
    }
    
    public void updateResistution(Double d) {
        resistution = d;
    }
    
    public void updateGravity(Double d) {
        gravity = new Vector(0, d);
    }
    
    public void updateTimeScale(Double d) {
        timeScale = d;
    }
    
    public void setMouseTool(MouseTool tool) {
        removeMouseListener(mouseTool);
        removeMouseMotionListener(mouseTool);
        mouseTool = tool;
        addMouseListener(mouseTool);
        addMouseMotionListener(mouseTool);
    }
    
    public void updateColor(Color c) {
        bodyColor = c;
        randomColor = false;
    }
    
    public void randomColor() {
        randomColor = true;
    }
    
    public interface MouseTool extends MouseMotionListener, MouseListener {
        
        public abstract void drawTool(Graphics g);
        
    }
    
    private interface BodyManipulationTool extends MouseTool {
        
        public abstract boolean bodySelected(RigidBody b);
        
        public abstract void interactWithBodyBeforeUpdate(double dt);
        
        public abstract void interactWithBodyAfterUpdate();
    }
    
    public class LinearDragTool implements MouseTool {
        private RigidBody selectedBody;
        private double[] COMrelativePos;
        
        public void mouseMoved(MouseEvent e) {}
        
        public void mouseDragged(MouseEvent e) {
            if(selectedBody == null)
                return;
            Point newCOM = new Point(e.getX() + COMrelativePos[0], e.getY() + COMrelativePos[1]);
            Point currentCOM = selectedBody.getCenterOfMass();
            selectedBody.translate(new Vector(newCOM.x - currentCOM.x, newCOM.y - currentCOM.y));
        }
        
        public void mouseExited(MouseEvent e) {}
        
        public void mouseEntered(MouseEvent e) {}
        
        public void mouseReleased(MouseEvent e) {
            if(selectedBody == null)
                return;
            selectedBody.setAngularVelocity(0);
            selectedBody.setVelocity(new Vector(0, 0));
            addBodyQueue.add(selectedBody);
            selectedBody.breakRest(false);
            selectedBody = null;
        }
        
        public void mouseClicked(MouseEvent e) {}
        
        public void mousePressed(MouseEvent e) {
            for(RigidBody b : bodies) {
                if(b.pointInsideBody(new Point(e.getX(), e.getY()))) {
                    selectedBody = b;
                    deleteQueue.add(b);
                    Point COM = b.getCenterOfMass();
                    COMrelativePos = new double[] {COM.x - e.getX(), COM.y - e.getY()};
                    break;
                }
            }
        }
        
        public void drawTool(Graphics g) {
            if(selectedBody != null)
                selectedBody.draw(g);
        }
        
    }
    
    public class ForceTool implements BodyManipulationTool {
        protected RigidBody selectedBody;
        protected Point selectedDragPoint;
        protected Point cursor = new Point(0, 0);
        private Point selectedBodyPreviousCOM;
        private double selectedBodyPreviousOrientation;
        
        public boolean bodySelected(RigidBody b) {
            return selectedBody == b;
        }
        
        public void mouseMoved(MouseEvent e) {
            cursor.x = e.getX();
            cursor.y = e.getY();
        }
        
        public void mouseDragged(MouseEvent e) {
            cursor.x = e.getX();
            cursor.y = e.getY();
        }
        
        public void mouseExited(MouseEvent e) {}
        
        public void mouseEntered(MouseEvent e) {}
        
        public void mouseReleased(MouseEvent e) {
            selectedBody = null;
        }
        
        public void mouseClicked(MouseEvent e) {}
        
        public void mousePressed(MouseEvent e) {
            for(RigidBody b : bodies) {
                if(b.pointInsideBody(new Point(e.getX(), e.getY()))) {
                    selectedBody = b;
                    selectedBodyPreviousCOM = b.getCenterOfMass();
                    selectedBodyPreviousOrientation = b.getOrientation();
                    selectedDragPoint = new Point(e.getX(), e.getY());
                    break;
                }
            }
        }
        
        public void interactWithBodyBeforeUpdate(double dt) {
            if(selectedBody == null)
            return;
            
            selectedBody.breakRest(false);
            double accelerationScale = 2;
            Vector v = new Vector(cursor.x - selectedDragPoint.x, cursor.y - selectedDragPoint.y);
            Vector impulse = v.multiplyByScalar(accelerationScale * dt * selectedBody.getMass());
            selectedBody.applyImpulseOnPoint(selectedDragPoint, impulse);
        }
        
        public void interactWithBodyAfterUpdate() {
            if(selectedBody == null)
            return;
            
            
            Point centerOfMass = selectedBody.getCenterOfMass();
              
            selectedDragPoint.x += centerOfMass.x - selectedBodyPreviousCOM.x;
            selectedDragPoint.y += centerOfMass.y - selectedBodyPreviousCOM.y;

            double theta = selectedBody.getOrientation() - selectedBodyPreviousOrientation;


            double rotX = Math.cos(theta) * (selectedDragPoint.x - centerOfMass.x) - Math.sin(theta) * (selectedDragPoint.y - centerOfMass.y);
            double rotY = Math.sin(theta) * (selectedDragPoint.x - centerOfMass.x) + Math.cos(theta) * (selectedDragPoint.y - centerOfMass.y);

            selectedDragPoint.x = centerOfMass.x + rotX;
            selectedDragPoint.y = centerOfMass.y + rotY;

            selectedBodyPreviousCOM = new Point(centerOfMass.x, centerOfMass.y);
            selectedBodyPreviousOrientation = selectedBody.getOrientation();
        }
        
        public void drawTool(Graphics g) {
            if(selectedBody == null)
            return;
            g.setColor(Color.BLACK);
            g.fillOval((int) selectedDragPoint.x - 6, (int) selectedDragPoint.y - 6, 12, 12);
            g.setColor(Color.WHITE);
            g.drawOval((int) selectedDragPoint.x - 4, (int) selectedDragPoint.y - 4, 8, 8);
            g.setColor(Color.white);
            g.drawLine((int) cursor.x, (int) cursor.y, (int) selectedDragPoint.x, (int) selectedDragPoint.y);
        }
    }
    
    public class ForceDragTool extends ForceTool {
        public void interactWithBodyBeforeUpdate(double dt) {
            if(selectedBody == null)
            return;
            
            selectedBody.breakRest(false);
        
            double accelerationScale = 200;
            Vector v = new Vector(cursor.x - selectedDragPoint.x, cursor.y - selectedDragPoint.y);
            Vector impulse = v.multiplyByScalar(accelerationScale * dt * selectedBody.getMass());
            selectedBody.applyImpulseOnPoint(selectedDragPoint, impulse);
            double dampen = 1 - 0.25 * dt/0.01;
            selectedBody.setVelocity(selectedBody.getVelocity().multiplyByScalar(dampen));
            selectedBody.setAngularVelocity(selectedBody.getAngularVelocity() * dampen);
        }
    }
    
    public class CustomBodyCreatorTool implements MouseTool {
        private ArrayList<RigidBodyPoint> customBodyPoints = new ArrayList<>();
        
        public void mouseMoved(MouseEvent e) {}
        
        public void mouseExited(MouseEvent e) {}
        
        public void mouseEntered(MouseEvent e) {}
        
        public void mouseReleased(MouseEvent e) {}
        
        public void mouseClicked(MouseEvent e) {}
        
        public void mouseDragged(MouseEvent e) {
            if(e.isMetaDown())
                return;
            if(customBodyPoints.isEmpty())
                customBodyPoints.add(new RigidBodyPoint(e.getX(), e.getY()));   
            else {
                Point p = customBodyPoints.get(customBodyPoints.size() - 1);
                if(Math.pow(p.x - e.getX(), 2) + Math.pow(p.y - e.getY(), 2) > 250)
                    customBodyPoints.add(new RigidBodyPoint(e.getX(), e.getY()));
            }
        }
        
        public void mousePressed(MouseEvent e) {
            randomizeColorIfNececary();
            if(e.isMetaDown()) { 
                if(customBodyPoints.size() > 2) {
                    RigidBodyPoint[] points = new RigidBodyPoint[customBodyPoints.size()];
                    for(int i = 0; i < points.length; i++) {
                        points[i] = customBodyPoints.get(i);
                    }
                    addBodyQueue.add(new RigidBody(points, new Vector(0, 0), 0, density, resistution, friction, createFixedBody, giveID(), bodyColor, drawCOM));
                }
                customBodyPoints.clear();
            }
            else
                customBodyPoints.add(new RigidBodyPoint(e.getX(), e.getY()));
        }
        
        public void drawTool(Graphics g) {
            if(customBodyPoints.size() > 1) {
                for(int i = 0; i < customBodyPoints.size() - 1; i++)
                    g.drawLine((int)customBodyPoints.get(i).x, (int)customBodyPoints.get(i).y, (int)customBodyPoints.get(i + 1).x, (int)customBodyPoints.get(i + 1).y);
            }
        }
    }
    
    public class RectangleCreatorTool implements MouseTool {
        private Point setPoint;
        private Point cursor;
        private RigidBodyPoint[] points;
        
        public void mouseMoved(MouseEvent e) {}
        
        public void mouseExited(MouseEvent e) {}
        
        public void mouseEntered(MouseEvent e) {}
        
        public void mouseReleased(MouseEvent e) {
            randomizeColorIfNececary();
            if(points != null)
                addBodyQueue.add(new RigidBody(points, new Vector(0, 0), 0, density, resistution, friction, createFixedBody, giveID(), bodyColor, drawCOM));
            points = null;
        }
        
        public void mouseClicked(MouseEvent e) {}
        
        public void mouseDragged(MouseEvent e) {
            cursor.x = e.getX();
            cursor.y = e.getY();
            points = new RigidBodyPoint[4];
            points[0] = new RigidBodyPoint(setPoint.x, setPoint.y);
            points[1] = new RigidBodyPoint(cursor.x, setPoint.y);
            points[2] = new RigidBodyPoint(cursor.x, cursor.y);
            points[3] = new RigidBodyPoint(setPoint.x, cursor.y);
        }
        
        public void mousePressed(MouseEvent e) {
            setPoint = new Point(e.getX(), e.getY());
            cursor = new Point(setPoint.x, setPoint.y);
        }
        
        public void drawTool(Graphics g) {
            try {
                if(points == null)
                    return;
                g.setColor(Color.white);
                for(int i = 0; i < points.length; i++) {
                    Point p1 = points[i];
                    Point p2 = (i == points.length - 1) ? points[0] : points[i + 1];
                    g.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
                }
            }
            catch(NullPointerException e) {
                
            }
        }
    }
    
    public class DeleteTool implements MouseTool {
        public void mouseMoved(MouseEvent e) {}
        
        public void mouseDragged(MouseEvent e) {}
        
        public void mouseExited(MouseEvent e) {}
        
        public void mouseEntered(MouseEvent e) {}
        
        public void mouseReleased(MouseEvent e) {}
        
        public void mouseClicked(MouseEvent e) {}
        
        public void mousePressed(MouseEvent e) {
            for(RigidBody b : bodies) {
                if(b.pointInsideBody(new Point(e.getX(), e.getY()))) {
                    b.breakRest(true);
                    deleteQueue.add(b);
                    break;
                }
            }
        }
        
        public void drawTool(Graphics g) {}
    }
    
    public class NSidedPolygonCreatorTool implements MouseTool {
        private Point center;
        private Point cursor;
        private RigidBodyPoint[] points;
        
        public void mouseMoved(MouseEvent e) {}
        
        public void mouseExited(MouseEvent e) {}
        
        public void mouseEntered(MouseEvent e) {}
        
        public void mouseReleased(MouseEvent e) {
            randomizeColorIfNececary();
            if(points != null)
                addBodyQueue.add(new RigidBody(points, new Vector(0, 0), 0, density, resistution, friction, createFixedBody, giveID(), bodyColor, drawCOM));
            points = null;
        }
        
        public void mouseClicked(MouseEvent e) {}
        
        public void mouseDragged(MouseEvent e) {
            cursor.x = e.getX();
            cursor.y = e.getY();
            double angInterval = 2 * Math.PI / nSides;

            points = new RigidBodyPoint[nSides];
            points[0] = new RigidBodyPoint(cursor.x, cursor.y);

            for(int i = 1; i < nSides; i++) {
                Point p = points[i - 1];
                double x = center.x + Math.cos(angInterval) * (p.x - center.x) - Math.sin(angInterval) * (p.y - center.y);
                double y = center.y + Math.sin(angInterval) * (p.x - center.x) + Math.cos(angInterval) * (p.y - center.y);
                points[i] = new RigidBodyPoint(x, y);
            }
        }
        
        public void mousePressed(MouseEvent e) {
            center = new Point(e.getX(), e.getY());
            cursor = new Point(center.x, center.y);
        }
        
        public void drawTool(Graphics g) {
            try {
                if(points == null)
                    return;
                g.setColor(Color.white);
                g.drawLine((int)center.x, (int)center.y, (int)cursor.x, (int)cursor.y);
                for(int i = 0; i < points.length; i++) {
                    Point p1 = points[i];
                    Point p2 = (i == points.length - 1) ? points[0] : points[i + 1];
                    g.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
                }
            }
            catch(NullPointerException e) {
                
            }
        }
    }   
    
    private void randomizeColorIfNececary() {
        if(randomColor) {
            switch((int) (Math.random() * 6)) {
                case 0:
                    bodyColor = Color.RED;
                    break;
                case 1:
                    bodyColor = Color.YELLOW;
                    break;
                case 2:
                    bodyColor = Color.GREEN;
                    break;
                case 3:
                    bodyColor = Color.BLUE;
                    break;
                case 4:
                    bodyColor = Color.MAGENTA;
                    break;
                case 5:
                    bodyColor = Color.ORANGE;
                    break;
            }
        }
    }
    
    public PhysicsPanel() {
        JFrame frame = new JFrame();
        frame.setSize(2000, 1050);
        frame.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE );
        frame.setContentPane(this);
        this.setVisible(true);
        frame.setVisible(true);
        this.addMouseListener(mouseTool);
        this.addMouseMotionListener(mouseTool);
        start();
    }
    
    public static void main(String[] args) {
        PhysicsPanel panel = new PhysicsPanel();
        panel.requestFocusInWindow();
        new UserInterfaceFrame(panel);
    }
    
}

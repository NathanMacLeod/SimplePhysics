/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author Nathan
 */
public class Point {
    public double x;
    public double y;
    
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public boolean equals(Object o) {
        if(!(o instanceof Point))
            return false;
        Point p = (Point) o;
        return p.x == x && p.y == y;
    }
    
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}

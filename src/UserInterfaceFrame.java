/*
 * File added by Nathan MacLeod 2019
 */
import javax.swing.*;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.GridLayout;
import javax.swing.event.ChangeListener;
import javax.swing.event.ChangeEvent;
import java.awt.event.*;
/**
 *
 * @author Nathan
 */
public class UserInterfaceFrame extends JFrame {
    private JSlider gravSlider;
    private JSlider densitySlider;
    private JSlider resistutionSlider;
    private JSlider frictionSlider;
    private JSlider timeScaleSlider;
    private JSlider numberOfSidesSlider;
    private JComboBox<String> mouseTools;
    private JComboBox<String> colorSelector;
    private JRadioButton fixedButton;
    private JRadioButton drawCOMButton;
    private JPanel nPanel;
    private JLabel nSides;
    private JLabel timeScale;
    private JLabel frictionCoefficient;
    private JLabel resistution;
    private UIListener listener;
    
    private class UIListener implements ActionListener, ChangeListener, WindowFocusListener{
        private PhysicsPanel physicsEngine;
        private JFrame frame;
        
        public UIListener(PhysicsPanel p, JFrame f) {
            physicsEngine = p;
            frame = f;
        }
        
        public void windowLostFocus(WindowEvent e) {
            frame.toFront();
        }
        
        public void windowGainedFocus(WindowEvent e) {
            
        }
        
        public void actionPerformed(ActionEvent e) {
            switch(((JComponent)e.getSource()).getName()) {
                case "Fixed":
                    physicsEngine.updateFixed(fixedButton.isSelected());
                    break;
                case "COM":
                    physicsEngine.updateDrawCOM(drawCOMButton.isSelected());
                    break;
                case "colorSelector":
                    String selection = (String)colorSelector.getSelectedItem();
                    Color c = null;
                    switch(selection) {
                        case "Red":
                            c = Color.RED;
                            break;
                        case "Orange":
                            c = Color.orange;
                            break;
                        case "Yellow":
                            c = Color.YELLOW;
                            break;
                        case "Green":
                            c = Color.GREEN;
                            break;
                        case "Blue":
                            c = Color.BLUE;
                            break;
                        case "Magenta":
                            c = Color.MAGENTA;
                            break;
                    }
                    if(c == null)
                        physicsEngine.randomColor();
                    else
                        physicsEngine.updateColor(c);
                    break;
                case "mouseTools":
                    selection = (String)mouseTools.getSelectedItem();
                    PhysicsPanel.MouseTool t = null;
                    nPanel.setVisible(false);
                    switch(selection) {
                        case "Rectangle":
                            t = physicsEngine.new RectangleCreatorTool();
                            break;
                        case "N Sided Polygon":
                            nPanel.setVisible(true);
                            t = physicsEngine.new NSidedPolygonCreatorTool();
                            break;
                        case "Custom Polygon":
                            t = physicsEngine.new CustomBodyCreatorTool();
                            break;
                        case "Force Tool":
                            t = physicsEngine.new ForceTool();
                            break;
                        case "Force Drag Tool":
                            t = physicsEngine.new ForceDragTool();
                            break;
                        case "Linear Drag Tool":
                            t = physicsEngine.new LinearDragTool();
                            break;
                        case "Delete Tool":
                            t = physicsEngine.new DeleteTool();
                            break;
                    }
                    physicsEngine.setMouseTool(t);
            }
            repaint();
        }
        
        public void stateChanged(ChangeEvent e) {
            switch(((JComponent)e.getSource()).getName()) {
                case "frictionSlider":
                    String val = ((Double)(frictionSlider.getValue()/100.0)).toString();
                    if(val.length() >= 5)
                        val = val.substring(0, 4);
                    frictionCoefficient.setText("Friction: " + val);
                    physicsEngine.updateFriction(frictionSlider.getValue()/100.0);
                    break;
                case "timeScaleSlider":
                    val = ((Double)(timeScaleSlider.getValue()/10.0)).toString();
                    if(val.length() >= 5)
                        val = val.substring(0, 4);
                    timeScale.setText("Time Multiplier: " + val);
                    physicsEngine.updateTimeScale(timeScaleSlider.getValue()/10.0);
                    break;
                case "gravSlider":
                    physicsEngine.updateGravity((double)gravSlider.getValue());
                    break;
                case "resistutionSlider":
                    val = ((Double)(resistutionSlider.getValue()/100.0)).toString();
                    if(val.length() >= 5)
                        val = val.substring(0, 4);
                    physicsEngine.updateResistution(resistutionSlider.getValue()/100.0);
                    resistution.setText("Resistution: " + val);
                    break;
                case "densitySlider":
                    physicsEngine.updateDensity((double)densitySlider.getValue());
                    break;
                case "numberOfSidesSlider":
                    nSides.setText("N: " + numberOfSidesSlider.getValue());
                    physicsEngine.updateNSides(numberOfSidesSlider.getValue());
                    break;
            }
            repaint();
        }
        
    }
    
    public UserInterfaceFrame(PhysicsPanel p) {
        //Declarations
        listener = new UIListener(p, this);
        this.addWindowFocusListener(listener);
        
        gravSlider = new JSlider(0, 1000, 300);
        gravSlider.setMajorTickSpacing(100);
        gravSlider.setMinorTickSpacing(50);
        gravSlider.setPaintTicks(true);
        gravSlider.setName("gravSlider");
        gravSlider.addChangeListener(listener);
        
        densitySlider = new JSlider(10, 100, 50);
        densitySlider.setMajorTickSpacing(10);
        densitySlider.setMinorTickSpacing(5);
        densitySlider.setPaintTicks(true);
        densitySlider.setName("densitySlider");
        densitySlider.addChangeListener(listener);
        
        resistutionSlider = new JSlider(1, 100, 30);
        resistutionSlider.setMajorTickSpacing(10);
        resistutionSlider.setMinorTickSpacing(5);
        resistutionSlider.setPaintTicks(true);
        resistutionSlider.setName("resistutionSlider");
        resistutionSlider.addChangeListener(listener);
        resistution = new JLabel("Resistution: 0.3");

        frictionSlider = new JSlider(0, 300, 50);
        frictionSlider.setMajorTickSpacing(50);
        frictionSlider.setMinorTickSpacing(10);
        frictionSlider.setPaintTicks(true);
        frictionSlider.setName("frictionSlider");
        frictionSlider.addChangeListener(listener);
        frictionCoefficient = new JLabel("Friction: 0.5");
                
        timeScaleSlider = new JSlider(0, 30, 10);
        timeScaleSlider.setMajorTickSpacing(10);
        timeScaleSlider.setMinorTickSpacing(2);
        timeScaleSlider.setPaintTicks(true);
        timeScaleSlider.addChangeListener(listener);
        timeScale = new JLabel("Time Multiplier: 1.0");
        timeScaleSlider.setName("timeScaleSlider");
        
        numberOfSidesSlider = new JSlider(3, 20, 3);
        numberOfSidesSlider.setMajorTickSpacing(1);
        numberOfSidesSlider.setPaintTicks(true);
        numberOfSidesSlider.addChangeListener(listener);
        nSides = new JLabel("N: 3");
        numberOfSidesSlider.setName("numberOfSidesSlider");
        
        mouseTools = new JComboBox(new String[] {"Force Tool", "Force Drag Tool", "Linear Drag Tool", "Delete Tool", "Rectangle", "N Sided Polygon", "Custom Polygon"});
        colorSelector = new JComboBox(new String[] {"Random", "Red", "Orange", "Yellow", "Green", "Blue", "Magenta"});
        colorSelector.setName("colorSelector");
        mouseTools.setName("mouseTools");
        mouseTools.addActionListener(listener);
        colorSelector.addActionListener(listener);
        
        fixedButton = new JRadioButton("Create Fixed Polygon");
        fixedButton.setName("Fixed");
        drawCOMButton = new JRadioButton("Draw Object Center Of Mass");
        drawCOMButton.setName("COM");
        fixedButton.addActionListener(listener);
        drawCOMButton.addActionListener(listener);
        
        //JFrame settings
        this.setSize(300, 400);
        this.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);//lol
        
        //Base && header
        JPanel base = new JPanel();
        setContentPane(base);
        base.setLayout(new BorderLayout());
        JPanel header = new JPanel();
        header.add(new JLabel("User Interface"));
        base.add(header, BorderLayout.NORTH);
        
        //Body, holds left and right panels
        JPanel body = new JPanel();
        base.add(body, BorderLayout.CENTER);
        body.setLayout(new GridLayout(1, 2));
        
        //Left side
        JPanel left = new JPanel();
        body.add(left);
        left.setLayout(new GridLayout(4, 1));
        
            //Mouse tools
        JPanel mousePanel = new JPanel();
        left.add(mousePanel);
        mousePanel.setLayout(new GridLayout(2, 1));
        mousePanel.add(new JLabel("Mouse Tools"));
        mousePanel.add(mouseTools);
        
            //Density slider
        JPanel densityPanel = new JPanel();
        left.add(densityPanel);
        densityPanel.setLayout(new GridLayout(2, 1));
        densityPanel.add(new JLabel("Density"));
        densityPanel.add(densitySlider);
            
            //Resitution slider
        JPanel resistutionPanel = new JPanel();
        left.add(resistutionPanel);
        resistutionPanel.setLayout(new GridLayout(4, 1));
        resistutionPanel.add(new JLabel("Resistution (Bounce)"));
        resistutionPanel.add(resistutionSlider);
        resistutionPanel.add(resistution);
        resistutionPanel.add(new JLabel("Buggy if > 0.7"));
        
            //Friction Slider
        JPanel frictionPanel = new JPanel();
        left.add(frictionPanel);
        frictionPanel.setLayout(new GridLayout(3, 1));
        frictionPanel.add(new JLabel("Friction"));
        frictionPanel.add(frictionSlider);
        frictionPanel.add(frictionCoefficient);
        
        //Right side
        JPanel right = new JPanel();
        body.add(right);
        right.setLayout(new GridLayout(5, 1));
        
            //Time Slider
        JPanel timePanel = new JPanel();
        right.add(timePanel);
        timePanel.setLayout(new GridLayout(3, 1));
        timePanel.add(new JLabel("Time Scale"));
        timePanel.add(timeScaleSlider);
        timePanel.add(timeScale);
        
            //Color selector
        JPanel colorPanel = new JPanel();
        right.add(colorPanel);
        colorPanel.setLayout(new GridLayout(2, 1));
        colorPanel.add(new JLabel("Color"));
        colorPanel.add(colorSelector);    
        
            //gravity slider
        JPanel gravPanel = new JPanel();
        right.add(gravPanel);
        gravPanel.setLayout(new GridLayout(2, 1));
        gravPanel.add(new JLabel("Gravity"));
        gravPanel.add(gravSlider);
                
            //Buttons
        JPanel buttonPanel = new JPanel();
        right.add(buttonPanel);
        buttonPanel.setLayout(new GridLayout(2, 1));
        buttonPanel.add(fixedButton);
        buttonPanel.add(drawCOMButton);
            //NSides
        nPanel = new JPanel();
        right.add(nPanel);
        nPanel.setLayout(new GridLayout(3, 1));
        nPanel.add(new JLabel("Number of Sides"));
        nPanel.add(numberOfSidesSlider);
        nPanel.add(nSides);
        nPanel.setVisible(false);
        setVisible(true);
    }
    
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public final class BeakXBoxController {

    private static final double kDeadband = .05;

    private XboxController controller;

    public JoystickButton a;
    public JoystickButton b;
    public JoystickButton x;
    public JoystickButton y;
    public JoystickButton start;
    public JoystickButton back;
    public JoystickButton left_bumper;
    public JoystickButton right_bumper;
    public JoystickButton left_stick_button;

    public BeakXBoxController(int port){
        controller = new XboxController(port);
        a = new JoystickButton(controller, 1);
        b = new JoystickButton(controller, 2);
        x = new JoystickButton(controller, 3);
        y = new JoystickButton(controller, 4);
        left_bumper = new JoystickButton(controller, 5);
        right_bumper = new JoystickButton(controller, 6);
        back = new JoystickButton(controller, 7);
        start = new JoystickButton(controller, 8);
        left_stick_button = new JoystickButton(controller, 9);
       
       
    }



    public double getLeftXAxis(){
        return controller.getRawAxis(0);
    }

    public double getLeftYAxis(){
        return controller.getRawAxis(1);
    }

    public double getRightXAxis(){
        return controller.getRawAxis(4);

    }

    public double getRightYAxis(){
        return controller.getRawAxis(5);
    }

    public double getLeftTrigger(){
        return controller.getRawAxis(2);
    }

    public double getRightTrigger(){
        return controller.getRawAxis(3);
    }

   

}
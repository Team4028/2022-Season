// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;

public class TraversalBarClimb extends SequentialCommandGroup {
  Climber climber = Climber.getInstance();
  /** Add your docs here. */
  public TraversalBarClimb() {
    addCommands(/*new MoveArm(-.8, 8), 
    new WaitCommand(.25),
    new MoveArm(0.25, 45), 
    new WaitCommand(.25), 
    new MoveArm(0.8, 60),  
    new WaitCommand(.5), 
    new ToggleGrippy(),  
    new WaitCommand(3), 
    new MoveArm(0.8, 180),  
    new WaitCommand(1),  
    new ToggleGrippy(),  
    new WaitCommand(1), 
    new MoveArm(-0.8, 30), 
    new WaitCommand(.25), 
    new MoveArm(0.25, 70), 
    new WaitCommand(.25),  
    new MoveArm(-0.8, 15), 
    new WaitCommand(.25), 
    new MoveArm(0.25, 40),*/
    new HighBarClimb(),
    new WaitCommand(.25), 
    new MoveArm(0.8, 60),  
    new WaitCommand(.5), 
    new ToggleGrippy(),  
    new WaitCommand(3), 
    new MoveArm(0.8, 180),  
    new WaitCommand(1),  
    new ToggleGrippy(),  
    new WaitCommand(1), 
    new MoveArm(-0.8, 30), 
    new WaitCommand(.25), 
    new MoveArm(0.25, 70), 
    new WaitCommand(.25),  
    new MoveArm(-0.8, 15), 
    new WaitCommand(.25));  
    //instant command for solenoids
    //,command,new command, new command, new command
  }


}

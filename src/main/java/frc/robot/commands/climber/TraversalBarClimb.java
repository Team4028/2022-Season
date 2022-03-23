// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VBusConstants;
import frc.robot.subsystems.Climber;

public class TraversalBarClimb extends SequentialCommandGroup {
  Climber climber = Climber.getInstance();
  /** Add your docs here. */
  public TraversalBarClimb() {
    addCommands(
    // new HighBarClimb(),
    new WaitCommand(.25), 
    new ToggleGrippy(),  
    new WaitCommand(3), 
    new MoveArm(VBusConstants.kClimberFast, 156.6),//135),  
    new WaitCommand(1),  
    new ToggleGrippy(),  
    new WaitCommand(.5), 
    new MoveArm(-VBusConstants.kClimberFast, -11.6),//-10), 
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 17.4),//15), 
    new WaitCommand(.25),  
    new MoveArm(-VBusConstants.kClimberFast, -34.8));//-30));  
    //instant command for solenoids
    //,command,new command, new command, new command
  }


}

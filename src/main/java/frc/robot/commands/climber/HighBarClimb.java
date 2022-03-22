// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VBusConstants;

public class HighBarClimb extends SequentialCommandGroup {
  /** Add your docs here. */
  public HighBarClimb() {
    addCommands(new MoveArm(-.8, -40), 
    new WaitCommand(.25),
    new MoveArm(VBusConstants.kClimberSlow, 29),//25), 
    new WaitCommand(.25), 
    new ToggleGrippy(),  
    new WaitCommand(3), 
    new MoveArm(VBusConstants.kClimberFast, 156.6),//135),  
    new WaitCommand(1),  
    new ToggleGrippy(),  
    new WaitCommand(1), 
    new MoveArm(-VBusConstants.kClimberFast, 5.8),//5), 
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 34.8),//30), 
    new WaitCommand(.25),  
    new MoveArm(-VBusConstants.kClimberFast, -46.4),//-40), 
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 34.8));//30));  
    //instant command for solenoids
    //,command,new command, new command, new command
  }


}

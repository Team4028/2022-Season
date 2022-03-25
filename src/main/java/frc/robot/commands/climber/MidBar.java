// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VBusConstants;

public class MidBar extends SequentialCommandGroup {
  /** Add your docs here. */
  public MidBar() {
    addCommands(new MoveArm(-.8, 15), // Pulls down to get tippy above
    new WaitCommand(.25),
    new MoveArm(VBusConstants.kClimberSlow, 80)); // slowly pulls grippy up to get tippy on the bar
    // new WaitCommand(.5), 
    // new ToggleGrippy(), // get grippy ready to latch
    // new WaitCommand(1), 
    // new MoveArm(VBusConstants.kClimberFast, 140), //get grippy to high
    // new WaitCommand(1)); 
    /*new ToggleGrippy(), // latch to high
    new WaitCommand(1));
    new MoveArm(-VBusConstants.kClimberFast, -5), // pull down to high
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 30), //slowly up until tippy clears
    new WaitCommand(.25),  
    new MoveArm(-VBusConstants.kClimberFast, -30), // pulls down until tippy releases
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 20)); // place robot get grippy above*/
    //instant command for solenoids
    //,command,new command, new command, new command
  }


}

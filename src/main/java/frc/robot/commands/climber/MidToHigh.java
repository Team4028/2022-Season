// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VBusConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidToHigh extends SequentialCommandGroup {
  /** Creates a new MidToHigh. */
  public MidToHigh() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ToggleGrippy(), // get grippy ready to latch
    new WaitCommand(1), 
    new MoveArm(VBusConstants.kClimberFast, 152),
    new WaitCommand(1),
    new ToggleGrippy()); //get grippy to high
    /*new ToggleGrippy(), // latch to high
    new WaitCommand(1),
    new MoveArm(-VBusConstants.kClimberFast, -5), // pull down to high
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 30), //slowly up until tippy clears
    new WaitCommand(.25),  
    new MoveArm(-VBusConstants.kClimberFast, -30), // pulls down until tippy releases
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 20));*/
  }
}


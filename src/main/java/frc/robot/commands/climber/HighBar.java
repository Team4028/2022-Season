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
public class HighBar extends SequentialCommandGroup {
  /** Creates a new HighBar. */
  public HighBar() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new MoveArm(-VBusConstants.kClimberFast, 50), // pull down to high
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 75), //slowly up until tippy clears
    new WaitCommand(.25),  
    new MoveArm(-VBusConstants.kClimberFast, 20), // pulls down until tippy releases
    new WaitCommand(.25), 
    new MoveArm(VBusConstants.kClimberSlow, 80));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightZeroSequence extends SequentialCommandGroup {
  /** Creates a new CurrentZeroSequence. */
  public RightZeroSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RightCurrentZero(),
      new InstantCommand(() -> Climber.getInstance().setRightEncoder(0.)),
      new MoveRightArm(0.1, 8.5),
      new InstantCommand(() -> Climber.getInstance().setRightEncoder(0.))
    );
  }
}

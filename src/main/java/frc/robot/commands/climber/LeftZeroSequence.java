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
public class LeftZeroSequence extends SequentialCommandGroup {
  /** Creates a new CurrentZeroSequence. */
  public LeftZeroSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LeftCurrentZero(),
      new InstantCommand(() -> Climber.getInstance().setLeftEncoder(0.)),
      new MoveLeftArm(0.1, 9),
      new InstantCommand(() -> Climber.getInstance().setLeftEncoder(0.))
    );
  }
}

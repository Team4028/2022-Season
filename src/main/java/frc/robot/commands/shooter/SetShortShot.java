// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShortShot extends SequentialCommandGroup {
  /** Creates a new SetShortShot. */
  public SetShortShot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> Shooter.getInstance().incrementCounter())
    ,new InstantCommand(() -> Shooter.getInstance().setLongshot(false))
    ,new InstantCommand(() -> Shooter.getInstance().setManualIndex())
    ,new WaitCommand(3.0)
    ,new InstantCommand(() -> Shooter.getInstance().resetCounter()));
  }
}

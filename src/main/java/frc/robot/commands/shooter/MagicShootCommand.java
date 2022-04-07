// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.chassis.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.conveyor.RunConveyorTwoBall;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MagicShootCommand extends SequentialCommandGroup {
  /** Creates a new MagicShootCommand. */
  RunShooterMotors _RunShooterMotors;
  public MagicShootCommand() {
    // Add your commands in the addCommands() call, e.g.
    _RunShooterMotors = new RunShooterMotors();
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateDrivetrainByLimelightAngle(false),
      new AcceptLimelightDistance(),
        sequence(
          new WaitCommand(0.25),
          new RunConveyorTwoBall(),
          new WaitCommand(0.25)
        ).deadlineWith(
          new RunShooterMotors(),
          new RotateDrivetrainByLimelightAngle(true)
        )
    );
  }
}

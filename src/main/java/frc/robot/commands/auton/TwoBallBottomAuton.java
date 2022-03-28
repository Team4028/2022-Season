// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util;
import frc.robot.commands.chassis.RotateDrivetrainToAngle;
import frc.robot.commands.conveyor.RunConveyor;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallBottomAuton extends SequentialCommandGroup {
  /** Creates a new TwoBallBottomAuton. */
  public TwoBallBottomAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Infeed.getInstance().setInfeedDown()),
      new InstantCommand(() -> Infeed.getInstance().runInfeedSingulatorMotors(1.0)),
      util.getPathPlannerSwerveControllerCommand(Trajectories.TwoBall_Bottom()),
      new RotateDrivetrainToAngle(Rotation2d.fromDegrees(79.0)),
      new InstantCommand(() -> Shooter.getInstance().runShooterMotors()),
      new WaitCommand(0.5),
      new RunConveyor().withTimeout(1.5)      
    );
  }
}

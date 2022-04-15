// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util;
import frc.robot.commands.BeakAutonCommand;
import frc.robot.commands.chassis.RotateDrivetrainToAngle;
import frc.robot.commands.conveyor.RunConveyor;
import frc.robot.commands.shooter.ResetDefaultIndex;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallMiddleAuton extends BeakAutonCommand {
  /** Creates a new TwoBallMiddleAuton. */
  public TwoBallMiddleAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super.addCommands(
      new ResetDefaultIndex(),
      new InstantCommand(() -> Infeed.getInstance().setInfeedDown()),
      new WaitCommand(0.25),
      new InstantCommand(() -> Infeed.getInstance().forceRunInfeed()),
      util.getPathPlannerSwerveControllerCommand(Trajectories.TwoBall_Middle()),
      new RotateDrivetrainToAngle(Rotation2d.fromDegrees(35.0)),
      new InstantCommand(() -> Shooter.getInstance().runShooterMotors()),
      new WaitCommand(0.5),
      new RunConveyor().withTimeout(1.5),
      new InstantCommand(() -> Shooter.getInstance().stop()),
      new InstantCommand(() -> Infeed.getInstance().stopInfeedSingulatorMotors())
    );
    super.setInitialPose(Trajectories.TwoBall_Middle());
  }
}

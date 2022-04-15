// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util;
import frc.robot.Constants.VBusConstants;
import frc.robot.commands.BeakAutonCommand;
import frc.robot.commands.chassis.RotateDrivetrainToAngle;
import frc.robot.commands.conveyor.RunConveyor;
import frc.robot.commands.conveyor.RunConveyorTwoBall;
import frc.robot.commands.infeed.ToggleInfeedUp;
import frc.robot.commands.shooter.ResetDefaultIndex;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallBackupAuton extends BeakAutonCommand {
  /** Creates a new FiveBallBackupAuton. */
  public FiveBallBackupAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super.setInitialPose(Trajectories.FiveBall_AcquireFirstCargo());
    super.addCommands(
      new InstantCommand(() -> Infeed.getInstance().setInfeedDown()),
      new WaitCommand(0.25),
      new InstantCommand(() -> Infeed.getInstance().forceRunInfeed()),
      new InstantCommand(() -> Shooter.getInstance().setShooterIndex(17.0)),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireFirstCargo()),
      new WaitCommand(0.2),
      new ToggleInfeedUp(),
      new InstantCommand(() -> Infeed.getInstance().stopInfeedSingulatorMotors()),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_GoToFirstShot())
      .alongWith(new WaitCommand(1.0).andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
      new RotateDrivetrainToAngle(Rotation2d.fromDegrees(38)).withTimeout(1.0),
      // new WaitCommand(1.1).deadlineWith(
      //   new RunConveyor()
      // ),
      new RunConveyorTwoBall(),
      new ResetDefaultIndex(),
      new ToggleInfeedUp(),
      new InstantCommand(() -> Shooter.getInstance().stop()),
      new InstantCommand(() -> Infeed.getInstance().forceRunInfeed(), Infeed.getInstance()),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireSecondCargo()).alongWith(new InstantCommand(() -> Shooter.getInstance().runShooterMotors())),
      // new WaitCommand(1.0).deadlineWith(
      //   new RunConveyor()
      // ),
      new RunConveyorTwoBall(),
      new InstantCommand(() -> Shooter.getInstance().stop()),
      new InstantCommand(() -> Infeed.getInstance().forceRunInfeed(), Infeed.getInstance()),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireLoadingZoneCargo()),
      new WaitCommand(0.35),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FourBall_Backup()),
      new WaitCommand(1.0),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FourBall_BackupReturnToShoot())
      .alongWith(new WaitCommand(1.0).andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
      new RotateDrivetrainToAngle(Rotation2d.fromDegrees(37.5)).withTimeout(1.0),
      // new WaitCommand(1.5).deadlineWith(
      //   new RunConveyor()
      // ),
      new RunConveyorTwoBall(),
      new WaitCommand(0.25),
      new InstantCommand(() -> Shooter.getInstance().stop()),
      new InstantCommand(() -> Vision.getInstance().setInfeedCamera())
      );
  }
}

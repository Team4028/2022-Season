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
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
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
      new InstantCommand(() -> Infeed.getInstance().runInfeedSingulatorMotors(1.0)),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireFirstCargo())
      .alongWith(new WaitCommand(1.5).andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
      //new InstantCommand(() -> Shooter.getInstance().setShooterIndex(16.5)),
      new RotateDrivetrainToAngle(Rotation2d.fromDegrees(35.0)),
      new InstantCommand(() -> Conveyor.getInstance().runConveyorMotor(VBusConstants.kConveyAll * 1.5)),
      new WaitCommand(1.1),
      new InstantCommand(() -> Conveyor.getInstance().stopConveyorMotor()),
      // new InstantCommand(() -> Shooter.getInstance().setShooterIndex(12.5)),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireSecondCargo()),
      new InstantCommand(() -> Conveyor.getInstance().runConveyorMotor(VBusConstants.kConveyAll * 1.5)),
      new WaitCommand(1.1),
      new InstantCommand(() -> Conveyor.getInstance().stopConveyorMotor()),
      new InstantCommand(() -> Shooter.getInstance().stop()),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FiveBall_AcquireLoadingZoneCargo()),
      new WaitCommand(0.75),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FourBall_Backup()),
      new WaitCommand(1.5),
      util.getPathPlannerSwerveControllerCommand(Trajectories.FourBall_ReturnToShoot())
      .alongWith(new WaitCommand(1.5).andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
      new InstantCommand(() -> Conveyor.getInstance().runConveyorMotor(VBusConstants.kConveyAll * 1.5)),
      new WaitCommand(1.1),
      new InstantCommand(() -> Conveyor.getInstance().stopConveyorMotor()),
      new WaitCommand(0.25),
      new InstantCommand(() -> Shooter.getInstance().stop()));
  }
}

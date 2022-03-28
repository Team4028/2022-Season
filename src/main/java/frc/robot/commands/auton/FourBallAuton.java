// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.time.Instant;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.chassis.RotateDrivetrainToAngle;
import frc.robot.commands.chassis.RotateDrivetrainToOdometryTargetAngle;
import frc.robot.commands.conveyor.RunConveyor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAuton extends SequentialCommandGroup {
  /** Creates a new TestAutonCommand. */
  public FourBallAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Infeed.getInstance().setInfeedDown()),
      new WaitCommand(0.1),
      getPathPlannerSwerveControllerCommand(Trajectories.FourBall_AcquireFirstCargo()).alongWith(new InstantCommand(() -> Infeed.getInstance().runInfeedSingulatorMotors(1.0))),
      new WaitCommand(0.5),
      new RotateDrivetrainToAngle(Rotation2d.fromDegrees(37.0)).alongWith(new InstantCommand(() -> Shooter.getInstance().runShooterMotors())),
      new WaitCommand(1.1).deadlineWith(new RunConveyor()),
      getPathPlannerSwerveControllerCommand(Trajectories.FourBall_AcquireLoadingZoneCargo()).alongWith(new WaitCommand(0.5).andThen(new InstantCommand(() -> Shooter.getInstance().stop()))),
      new WaitCommand(1.5),
      getPathPlannerSwerveControllerCommand(Trajectories.FourBall_ReturnToShoot()).alongWith(new WaitCommand(0.5).andThen(new InstantCommand(() -> Shooter.getInstance().runShooterMotors()))),
      new RotateDrivetrainToAngle(Rotation2d.fromDegrees(35.0)),
      //new RotateDrivetrainToOdometryTargetAngle(),
      new WaitCommand(1.1).deadlineWith(new RunConveyor()),
      new WaitCommand(0.25).andThen(new InstantCommand(() -> Shooter.getInstance().stop())),
      new InstantCommand(() -> Infeed.getInstance().stopInfeedSingulatorMotors()),
      new RotateDrivetrainToOdometryTargetAngle().withTimeout(2.5)
    );
  }
  public static Pose2d initialPose(){
    return Trajectories.FourBall_AcquireFirstCargo().getInitialPose();
  }
  public Command getPathPlannerSwerveControllerCommand(PathPlannerTrajectory traj) {
    return new PPSwerveControllerCommand(
        traj,
        DriveSubsystem.getInstance()::getPose,
        DriveConstants.kDriveKinematics,
        AutoConstants.AUTON_X_CONTROLLER,
        AutoConstants.AUTON_Y_CONTROLLER,
        AutoConstants.AUTON_THETA_CONTROLLER,
        DriveSubsystem.getInstance()::setModuleStates,
        DriveSubsystem.getInstance())
            .andThen(new InstantCommand(() -> DriveSubsystem.getInstance().drive(0, 0, 0, true)));
  }
}

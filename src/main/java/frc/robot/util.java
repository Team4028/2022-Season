// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public final class util {
    public static double toFalconRPM(double velocity) {
        return velocity * 600 / 4096;
    }

    public static double toFalconVelocity(double rpm) {
        return rpm * 4096 / 600;
    }
    public static double deadband(double input){
        return Math.abs(input) > 0.025? input: 0.0;
    }
    public static Command getPathPlannerSwerveControllerCommand(PathPlannerTrajectory traj) {
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

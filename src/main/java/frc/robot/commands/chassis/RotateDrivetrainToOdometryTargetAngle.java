// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateDrivetrainToOdometryTargetAngle extends ProfiledPIDCommand {
    /** Creates a new RotateDrivetrainToOdometryTargetAngle. */
    public RotateDrivetrainToOdometryTargetAngle() {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        AutoConstants.kPThetaController,
                        0.0,
                        0.0,
                        // The motion profile constraints
                        AutoConstants.kThetaControllerConstraints),
                // This should return the measurement
                () -> DriveSubsystem.getInstance().getPose().getRotation().getRadians(),
                // This should return the goal (can also be a constant)
                () -> DriveSubsystem.getInstance().getOdometryAngleToTarget().getRadians(),
                // This uses the output
                (output, setpoint) -> {
                    // Use the output (and setpoint, if desired) here
                    DriveSubsystem.getInstance().setModuleStates(
                            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            0.0,
                                            0.0,
                                            output + setpoint.velocity,
                                            DriveSubsystem.getInstance().getPose().getRotation())));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(DriveSubsystem.getInstance());
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Units.degreesToRadians(1.0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}

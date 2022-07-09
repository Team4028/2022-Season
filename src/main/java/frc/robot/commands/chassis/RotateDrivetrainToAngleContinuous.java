// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateDrivetrainToAngleContinuous extends ProfiledPIDCommand {
    /** Creates a new RotateDrivetrainToAngleContinuous. */
    public RotateDrivetrainToAngleContinuous(Supplier<Rotation2d> goal, DoubleSupplier leftXSpeed,
            DoubleSupplier leftYSpeed) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        7.55,
                        0,
                        0.18,
                        // The motion profile constraints
                        new TrapezoidProfile.Constraints(AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)),
                // This should return the measurement
                () -> DriveSubsystem.getInstance().getPose().getRotation().getRadians(),
                // This should return the goal (can also be a constant)
                () -> goal.get().getRadians(),
                // This uses the output
                (output, setpoint) -> {
                    // Use the output (and setpoint, if desired) here
                    DriveSubsystem.getInstance().setModuleStates(
                            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            DriveConstants.i_kMaxSpeedMetersPerSecond * leftYSpeed.getAsDouble(),
                                            DriveConstants.i_kMaxSpeedMetersPerSecond * leftXSpeed.getAsDouble(),
                                            output + setpoint.velocity,
                                            DriveSubsystem.getInstance().getPose().getRotation())));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        // addRequirements(DriveSubsystem.getInstance());
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Units.degreesToRadians(0.75));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // return getController().atGoal();
    }
}

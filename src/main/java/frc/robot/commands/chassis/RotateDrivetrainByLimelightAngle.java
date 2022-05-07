// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateDrivetrainByLimelightAngle extends ProfiledPIDCommand {
    /** Creates a new RotateDrivetrainByLimelightAngle. */
    private boolean continuous;

    public RotateDrivetrainByLimelightAngle(boolean continuous) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        AutoConstants.kPThetaController * .85,
                        0.0,
                        0.0,
                        // The motion profile constraints
                        AutoConstants.kThetaControllerConstraints),
                // This should return the measurement
                () -> Limelight.getInstance().getXForRot() * Math.PI / 180,
                // This should return the goal (can also be a constant)
                () -> 0.0,
                // This uses the output
                (output, setpoint) -> {
                    // Use the output (and setpoint, if desired) here
                    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    DriveConstants.i_kMaxSpeedMetersPerSecond
                                            * RobotContainer.getInstance().getSpeedScaledDriverLeftY(),
                                    DriveConstants.i_kMaxSpeedMetersPerSecond
                                            * RobotContainer.getInstance().getSpeedScaledDriverLeftX(),
                                    output + setpoint.velocity,
                                    DriveSubsystem.getInstance().getPose().getRotation()));
                    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.i_kMaxSpeedMetersPerSecond);
                    DriveSubsystem.getInstance().setModuleStates(states);
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(DriveSubsystem.getInstance(), Limelight.getInstance());
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Units.degreesToRadians(1.0));
        this.continuous = continuous;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!continuous) {
            return getController().atGoal();
        } else {
            return false;
        }
    }
}

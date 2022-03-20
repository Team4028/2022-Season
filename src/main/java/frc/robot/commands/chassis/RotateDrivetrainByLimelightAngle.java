// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateDrivetrainByLimelightAngle extends ProfiledPIDCommand {
  /** Creates a new RotateDrivetrainByLimelightAngle. */
  public RotateDrivetrainByLimelightAngle() {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            AutoConstants.kPThetaController * 1.5,
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
          DriveSubsystem.getInstance().drive(0.0, 0.0, output, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(DriveSubsystem.getInstance(), Limelight.getInstance());
    getController().enableContinuousInput(-Math.PI, Math.PI);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Limelight.getInstance().getX()) < 1.0;
  }
}

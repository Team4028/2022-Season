// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateDriveTrain extends ProfiledPIDCommand {
  /** Creates a new RotateDriveTrain. */
  public RotateDriveTrain(Rotation2d rotation) {
    super(
        // The ProfiledPIDController used by the command
        AutoConstants.AUTON_THETA_CONTROLLER,
        // This should return the measurement
        () -> DriveSubsystem.get_instance().getPose().getRotation().getRadians(),
        // This should return the goal (can also be a constant)
        () -> DriveSubsystem.get_instance().getPose().getRotation().minus(rotation).getRadians(),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
        });
        addRequirements(Limelight.getInstance(), DriveSubsystem.get_instance());
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

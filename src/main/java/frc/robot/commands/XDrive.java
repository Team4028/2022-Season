// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class XDrive extends CommandBase {
  /** Creates a new XDrive. */
  private DriveSubsystem m_drive = DriveSubsystem.get_instance();
  /**
   * Turns Swerve Module Wheels to an "X" Position in order to resist pushing and hold position
   */
  public XDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setModuleStates(new SwerveModuleState[]{
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

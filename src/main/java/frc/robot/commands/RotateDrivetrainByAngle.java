// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDrivetrainByAngle extends CommandBase {
  /** Creates a new RotateDrivetrainByAngle. */
  private DriveSubsystem m_drive = DriveSubsystem.get_instance();
  private Rotation2d rotation;
  private boolean fieldRelative;
  private ProfiledPIDController thetaController =
  new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  public RotateDrivetrainByAngle(Rotation2d rotation, boolean fieldOriented) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.fieldRelative = fieldOriented;
    this.rotation = rotation;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.drive(0, 0, thetaController.calculate(m_drive.getPose().getRotation().getRadians(),
    rotation.getRadians()), fieldRelative);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(0, 0, thetaController.calculate(m_drive.getPose().getRotation().getRadians(),
    rotation.getRadians()), fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, fieldRelative);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotation.minus(m_drive.getPose().getRotation()).getDegrees() < 1;
  }
}

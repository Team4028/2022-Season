// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDrivetrainByAngle extends CommandBase {
  /** Creates a new RotateDrivetrainByAngle. */
  private DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private Rotation2d rotation;
  private boolean fieldRelative;
  private Rotation2d rotinit;
  private ProfiledPIDController thetaController =
  new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  /**
   * Rotates Drivetrain to given angle, either field or robot oriented
   * @param rotation Demanded rotation
   * @param fieldOriented If true, rotates to given rotation for field, if false, rotates rotation for robot
   */
  public RotateDrivetrainByAngle(Rotation2d rotation, boolean fieldOriented) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.fieldRelative = fieldOriented;
    this.rotation = rotation;
    


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    rotinit = m_drive.getPose().getRotation();
    m_drive.drive(0, 0, thetaController.calculate(m_drive.getPose().getRotation().getRadians(),
    rotinit.rotateBy(rotation).getRadians()), fieldRelative);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(0, 0, thetaController.calculate(m_drive.getPose().getRotation().getRadians(),
    rotinit.rotateBy(rotation).getRadians()), fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, fieldRelative);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rotinit.rotateBy(rotation).minus(m_drive.getPose().getRotation()).getDegrees()) < 0.75;
  }
}

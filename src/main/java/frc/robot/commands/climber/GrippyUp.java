// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class GrippyUp extends CommandBase {
  private Climber climber = Climber.getInstance();
  /** Creates a new GrippyUp. */
  public GrippyUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.rightMotorForward(.8);
    climber.leftMotorForward(.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.leftMotorOff();
    climber.rightMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(climber.getLeftEncoderPosition() - 100) < 2);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  private Climber climber = Climber.getInstance();
  double speed;
  double encoderValue;
  public MoveArm(double speed, double encoderValue) {
    this.speed = speed;
    this.encoderValue = encoderValue;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.rightMotorForward(speed);
    climber.leftMotorForward(speed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.leftMotorOff();
    climber.rightMotorOff();
    System.out.println("Turning Off");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(climber.getLeftEncoderPosition() - encoderValue) < 2) {
      return true;
    } else {
      return false;
    }
  }
}

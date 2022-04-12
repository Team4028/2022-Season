// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class MoveRightArm extends CommandBase {
  /** Creates a new traversalBarCom. */
  private static Climber _i = Climber.getInstance();
  double _Speed;
  double _encoderValue;
  public MoveRightArm(double Speed, double encoderValue) {
    _Speed = Speed;
    _encoderValue = encoderValue;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _i.rightMotorForward(_Speed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _i.rightMotorOff();
    System.out.println("Turning Off");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(_i.getRightEncoderPosition() - _encoderValue) < 3.) {
      
      return true;
    } else {
      return false;
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RightCurrentZero extends CommandBase {
  /** Creates a new ZeroWithCurrent. */
  public RightCurrentZero() {
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Climber.getInstance().setRampRate(10.);
    Climber.getInstance().rightMotorForward(-.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Climber.getInstance().rightMotorForward(-.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climber.getInstance().rightMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    return Climber.getInstance().rightCurrent() > 5;
  }
}

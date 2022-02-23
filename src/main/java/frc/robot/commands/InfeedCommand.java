// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Infeed;

public class InfeedCommand extends CommandBase {
  /** Creates a new InfeedCommand. */
  private Infeed _Infeed = new Infeed();
  public InfeedCommand() {
    addRequirements(_Infeed);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _Infeed.runMotorInfeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Infeed.runMotorInfeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

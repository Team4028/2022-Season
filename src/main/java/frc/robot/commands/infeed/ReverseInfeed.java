// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Infeed;

public class ReverseInfeed extends CommandBase {
  /** Creates a new ReverseInfeed. */
  public ReverseInfeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Infeed.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Infeed.getInstance().runInfeedMotor(-1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Infeed.getInstance().runInfeedMotor(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Infeed.getInstance().stopInfeedSingulatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

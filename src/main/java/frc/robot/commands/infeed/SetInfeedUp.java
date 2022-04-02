// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Infeed;

public class SetInfeedUp extends CommandBase {
    private Infeed infeed = Infeed.getInstance();
  /** Creates a new SetInfeedUp. */
  public SetInfeedUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(infeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    infeed.setInfeedUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

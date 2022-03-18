// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class IncrementShooterIndex extends CommandBase {
  /** Creates a new IncrementShooterIndex. */
  private Shooter _s = Shooter.getInstance();
  private boolean fine;

  public IncrementShooterIndex(boolean fine) {
    addRequirements(_s);
    // Use addRequirements() here to declare subsystem dependencies.
    this.fine = fine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _s.incrementIndex(fine);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

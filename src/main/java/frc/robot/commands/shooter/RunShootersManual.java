// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RunShootersManual extends CommandBase {
  /** Creates a new RunShootersManual. */
  public RunShootersManual() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Shooter.getInstance(), Vision.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.getInstance().setShooterIndex(Shooter.getInstance().manualIndex(), true);
    Shooter.getInstance().runShooterMotors();
    Vision.getInstance().setShooterCamera();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.getInstance().setShooterIndex(Shooter.getInstance().manualIndex(), true);
    Shooter.getInstance().runShooterMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.getInstance().stop();
    Vision.getInstance().setInfeedCamera();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

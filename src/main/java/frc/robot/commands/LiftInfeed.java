// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SingulatorAndInfeed;

public class LiftInfeed extends CommandBase {
  /** Creates a new LiftInfeed. */
  private WaitCommand _wait;
  public LiftInfeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SingulatorAndInfeed.get_instance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SingulatorAndInfeed.get_instance().liftInfeed();
    _wait = new WaitCommand(3.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SingulatorAndInfeed.get_instance().holdInfeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _wait.isFinished();
  }
}

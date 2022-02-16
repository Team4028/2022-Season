// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestingEther;

public class RunInfeedSingulatorMotors extends CommandBase {
  /** Creates a new ToggleOneAndTwo. */
  private TestingEther _TestingEther = TestingEther.get_instance();
  public RunInfeedSingulatorMotors() {
    addRequirements(_TestingEther);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _TestingEther.runInfeedSingulatorMotors(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _TestingEther.runInfeedSingulatorMotors(1);
    System.out.println("line");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _TestingEther.stopInfeedSingulatorConveyorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SingulatorAndInfeed;


public class RunInfeedSingulatorMotors extends CommandBase {
  /** Creates a new ToggleOneAndTwo. */
  private SingulatorAndInfeed _SingulatorAndInfeed = SingulatorAndInfeed.get_instance();
  public RunInfeedSingulatorMotors() {
    addRequirements(_SingulatorAndInfeed);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _SingulatorAndInfeed.runInfeedSingulatorMotors(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _SingulatorAndInfeed.runInfeedSingulatorMotors(1);
    System.out.println("line");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _SingulatorAndInfeed.stopInfeedSingulatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VBusConstants;
import frc.robot.subsystems.TestingEther;

public class ReverseInfeedAndConveyor extends CommandBase {
  /** Creates a new ReverseInfeedAndConveyor. */
  private TestingEther _eth = TestingEther.get_instance();
  public ReverseInfeedAndConveyor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_eth);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _eth.runConveyorMotor(-VBusConstants.kConveyAll);
    _eth.runInfeedSingulatorMotors(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _eth.runConveyorMotor(-VBusConstants.kConveyAll);
    _eth.runInfeedSingulatorMotors(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _eth.stopConveyorMotor();
    _eth.stopInfeedSingulatorConveyorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

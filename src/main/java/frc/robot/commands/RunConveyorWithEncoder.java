// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.VBusConstants;
import frc.robot.subsystems.TestingEther;

public class RunConveyorWithEncoder extends CommandBase {
  private TestingEther _TE = TestingEther.get_instance();
  /** Creates a new RunWithEncoder. */
  public RunConveyorWithEncoder() {
    addRequirements(_TE);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _TE.runConveyorMotorWithEncoder(EncoderConstants.kConveyOne, VBusConstants.kConveyOne);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _TE.runConveyorMotorWithEncoder(EncoderConstants.kConveyOne, VBusConstants.kConveyOne);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _TE.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _TE.getIsTargetReached();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.VBusConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Infeed;

public class RunConveyorOneBallEvacuate extends CommandBase {
  /** Creates a new RunConveyorOneBallEvacuate. */
  Conveyor conveyor = Conveyor.getInstance();
  public RunConveyorOneBallEvacuate() {
    addRequirements(conveyor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.resetEncoder();
    conveyor.setIsTargetReached();
    conveyor.runConveyorMotorWithEncoder(EncoderConstants.kConveyOne, VBusConstants.kConveyOne);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.runConveyorMotorWithEncoder(EncoderConstants.kConveyOne, VBusConstants.kConveyOne);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return conveyor.getIsTargetReached();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.VBusConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Infeed;


public class RunConveyorTwoBall extends CommandBase {
  private Conveyor conveyor = Conveyor.getInstance();
  private Infeed infeed = Infeed.getInstance();
  /** Creates a new RunWithEncoderA. */
  public RunConveyorTwoBall() {
    addRequirements(conveyor, infeed);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.resetEncoder();
    conveyor.setIsTargetReached();
    conveyor.runConveyorMotorWithEncoder(EncoderConstants.kConveyTwo, VBusConstants.kConveyTwo);

    infeed.runSingulator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.runConveyorMotorWithEncoder(EncoderConstants.kConveyTwo, VBusConstants.kConveyTwo);
    infeed.runSingulator();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    infeed.stopInfeedSingulatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return conveyor.getIsTargetReached();
  }
}

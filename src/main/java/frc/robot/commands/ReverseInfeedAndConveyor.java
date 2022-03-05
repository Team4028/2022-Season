// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VBusConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Infeed;

public class ReverseInfeedAndConveyor extends CommandBase {
  /** Creates a new ReverseInfeedAndConveyor. */
  private Conveyor _Con = Conveyor.get_instance();
  private Infeed _Infeed = Infeed.get_instance();
  public ReverseInfeedAndConveyor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_Con);
    addRequirements(_Infeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _Con.runConveyorMotor(-VBusConstants.kConveyAll);
    _Infeed.runInfeedSingulatorMotors(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Con.runConveyorMotor(-VBusConstants.kConveyAll);
    _Infeed.runInfeedSingulatorMotors(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Con.stopConveyorMotor();
    _Infeed.stopInfeedSingulatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

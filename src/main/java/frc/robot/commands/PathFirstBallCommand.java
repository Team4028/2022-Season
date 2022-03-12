// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.Trajectories;

public class PathFirstBallCommand extends CommandBase {
  private DriveSubsystem m_drive = DriveSubsystem.get_instance();
  private Trajectories _trajectories = Trajectories.get_instance();
  private Trajectory _firstBallTrajectory = _trajectories.getTestCompFirstBall();
  private Command swerveControllerCommand;
  /** Creates a new PathFirstBallCommand. */
  public PathFirstBallCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    swerveControllerCommand = RobotContainer.get_instance().getSwerveControllerCommand(_firstBallTrajectory);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

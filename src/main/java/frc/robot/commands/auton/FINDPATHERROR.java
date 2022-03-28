// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class FINDPATHERROR extends CommandBase {
  /** Creates a new FINDPATHERROR. */
  private Translation2d demandPosM;
  private Trajectory traj;
  private Translation2d actualPosM;
  private int iterations = 0;
  private double timeElapsed;
  private double totalMeasurements;
  private double totalError;
  private double initTime;
  /**
   * Outputs the average error of robot's actual position from planned trajectory position to SmartDashboard
   * @param traj Trajectory being followed by the parallel SwerveControllerCommand
   */
  public FINDPATHERROR(Trajectory traj) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.traj = traj;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    iterations++;
    totalError = 0.0;
    totalMeasurements = 0.0;
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timeElapsed = Timer.getFPGATimestamp() - initTime;
    if(iterations == 50){
      demandPosM = traj.sample(timeElapsed).poseMeters.getTranslation();
      actualPosM = DriveSubsystem.getInstance().getPose().getTranslation();
      double err = demandPosM.getDistance(actualPosM);
      //SmartDashboard.putNumber("Current Err: " , Math.abs(err));
      totalMeasurements++;
      totalError += Math.abs(err);
      iterations = 0;
    }
    iterations++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putNumber("Average Err: ", totalError / totalMeasurements);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeElapsed > traj.getTotalTimeSeconds();
  }
}

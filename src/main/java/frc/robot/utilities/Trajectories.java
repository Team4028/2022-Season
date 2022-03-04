// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/** Add your docs here. */
public class Trajectories {
    private static DriveSubsystem m_drive = DriveSubsystem.get_instance();
    private static Trajectories _instance;
    public static Trajectories get_instance(){
        if(_instance == null){
            _instance = new Trajectories();
        }
        return _instance;
    }
    public Trajectory getaTestArcTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(),
    List.of(
      new Translation2d(1,1),
      new Translation2d(2,-1)
    ),
    new Pose2d(3, 0, Rotation2d.fromDegrees(180)),
    AutonTrajectoryConfig);

    public Trajectory getTestCompFirstBall(){
        return TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0.0,0.0,new Rotation2d(0)),
        new Pose2d(1.19, -0.12, Rotation2d.fromDegrees(0))),
        AutonTrajectoryConfig);
    }

    // public static final Trajectory TestCompTurnFirstShoot = TrajectoryGenerator.generateTrajectory(
    // List.of(new Pose2d(1.19, -0.12, Rotation2d.fromDegrees(0)),
    // new Pose2d(1.19, -0.12, Rotation2d.fromDegrees(180))),
    // AutonTrajectoryConfig);

    public Trajectory getTestCompSecondBall(){
        return TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(1.19, -0.12, Rotation2d.fromDegrees(0)),
        new Pose2d(4.18, -2.13, Rotation2d.fromDegrees(10))),
        AutonTrajectoryConfig);
    }

    public Trajectory getTestCompReturnShoot(){
        return TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(4.18, -2.13, Rotation2d.fromDegrees(10)),
        new Pose2d(1.16, -0.76, Rotation2d.fromDegrees(175))),
        AutonTrajectoryConfig);
    }



}

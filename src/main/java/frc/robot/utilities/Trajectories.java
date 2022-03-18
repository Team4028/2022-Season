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

import static frc.robot.Constants.AutoConstants.*;

/**
 * Class containing all Auton Trajectories
 */
public class Trajectories {
    private static Trajectories _instance;

    public static Trajectories getInstance() {
        if (_instance == null) {
            _instance = new Trajectories();
        }
        return _instance;
    }

    /**
     * Simple prototype trajectory for testing
     */
    public Trajectory getTestArcTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(),
            List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(180)),
            AutonTrajectoryConfig);

    // TODO: Write actual paths
    /**
     * Trajectory for first part of prototype Auton sequence
     * 
     * @return Trajectory that acquires first ball
     */
    public Trajectory getTestCompFirstBall() {
        return TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0.0, 0.0, new Rotation2d(0)),
                        new Pose2d(1.19, -0.12, Rotation2d.fromDegrees(0))),
                AutonTrajectoryConfig);
    }

    /**
     * Trajectory for second part of prototype Auton sequence
     * 
     * @return Trajectory that acquires second ball
     */
    public Trajectory getTestCompSecondBall() {
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.19, -0.12, Rotation2d.fromDegrees(187.0)),
                List.of(new Translation2d(2.19, -2.00)),
                new Pose2d(3.18, -2.13, Rotation2d.fromDegrees(10)),
                AutonTrajectoryConfig);
    }

    // TODO: Investigate weird behavior at start of return path
    /**
     * Trajectory for third (and final) part of prototype Auton sequence
     * 
     * @return Trajectory that returns robot to shooting range
     */
    public Trajectory getTestCompReturnShoot() {
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(3.18, -2.13, Rotation2d.fromDegrees(10)),
                List.of(new Translation2d(2.19, -1.13)),
                new Pose2d(1.19, -0.12, Rotation2d.fromDegrees(187)),
                AutonTrajectoryConfig);
    }

}

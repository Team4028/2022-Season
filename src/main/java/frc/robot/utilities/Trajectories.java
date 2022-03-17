// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.List;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import frc.robot.Constants.AutoConstants;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Class containing all Auton Trajectories
*/
public class Trajectories {
    private static Trajectories _instance;
    public static Trajectories get_instance(){
        if(_instance == null){
            _instance = new Trajectories();
        }
        return _instance;
    }

    /**
     * @return Path from start to first ball
     */
    public PathPlannerTrajectory FourBall_AcquireFirstCargo(){
        return PathPlanner.loadPath(
            "FourBall_AcquireFirstCargo",
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from first cargo to Loading Zone to infeed two cargo
     */
    public PathPlannerTrajectory FourBall_AcquireLoadingZoneCargo(){
        return PathPlanner.loadPath(
            "FourBall_AcquireLoadingZoneCargo",
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from Loading Zone to sweet spot for shot
     */
    public PathPlannerTrajectory FourBall_ReturnToShoot(){
        return PathPlanner.loadPath(
            "FourBall_ReturnToShoot",
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from start to first cargo then to shot spot (just behind second cargo)
     */
    public PathPlannerTrajectory FiveBall_AcquireFirstCargo(){
        return PathPlanner.loadPath(
            "FiveBall_AcquireFirstCargo",
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from first cargo to second cargo (short path ~ a foot forward)
     */
    public PathPlannerTrajectory FiveBall_AcquireSecondCargo(){
        return PathPlanner.loadPath(
            "FiveBall_AcquireSecondCargo",
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from second cargo to Loading Zone
     */
    public PathPlannerTrajectory FiveBall_AcquireLoadingZoneCargo(){
        return PathPlanner.loadPath(
            "FiveBall_AcquireLoadingZoneCargo",
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from Loading Zone to sweet spot shot
     */
    public PathPlannerTrajectory FiveBall_ReturnToShoot(){
        return PathPlanner.loadPath(
            "FiveBall_ReturnToShoot",
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );
    }
}

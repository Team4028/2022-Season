// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
     * @return Path from start to first ball
     */
    public static PathPlannerTrajectory FourBall_AcquireFirstCargo(){
        return PathPlanner.loadPath(
            "FourBall_AcquireFirstCargo",
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from first cargo to Loading Zone to infeed two cargo
     */
    public static PathPlannerTrajectory FourBall_AcquireLoadingZoneCargo(){
        return PathPlanner.loadPath(
            "FourBall_AcquireLoadingZoneCargo",
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from Loading Zone to sweet spot for shot
     */
    public static PathPlannerTrajectory FourBall_ReturnToShoot(){
        return PathPlanner.loadPath(
            "FourBall_ReturnToShoot",
        kMaxSpeedMetersPerSecond * 0.65,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from start to first cargo then to shot spot (just behind second cargo)
     */
    public static PathPlannerTrajectory FiveBall_AcquireFirstCargo(){
        return PathPlanner.loadPath(
            "FiveBall_AcquireFirstCargo",
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from first cargo to second cargo (short path ~ a foot forward)
     */
    public static PathPlannerTrajectory FiveBall_AcquireSecondCargo(){
        return PathPlanner.loadPath(
            "FiveBall_AcquireSecondCargo",
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from second cargo to Loading Zone
     */
    public static PathPlannerTrajectory FiveBall_AcquireLoadingZoneCargo(){
        return PathPlanner.loadPath(
            "FiveBall_AcquireLoadingZoneCargo",
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
    /**
     * @return Path from Loading Zone to sweet spot shot
     */
    public static PathPlannerTrajectory FiveBall_ReturnToShoot(){
        return PathPlanner.loadPath(
            "FiveBall_ReturnToShoot",
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
    public static PathPlannerTrajectory TwoBall_Top(){
        return PathPlanner.loadPath(
            "TwoBall_Top",
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
    public static PathPlannerTrajectory TwoBall_GetOutOfTheWay(){
        return PathPlanner.loadPath(
            "TwoBall_GetOutOfTheWay",
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
        );
    }
}

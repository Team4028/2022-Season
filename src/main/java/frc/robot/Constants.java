// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //TODO: Better organization of Constants class
  public static final class DriveConstants {

    public static final boolean MK4I = true;
    public static final double BASE_SPEED_SCALE = 0.25;

    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 8;

    public static final int i_kFrontLeftDriveMotorPort = 2;
    public static final int i_kRearLeftDriveMotorPort = 6;
    public static final int i_kFrontRightDriveMotorPort = 4;
    public static final int i_kRearRightDriveMotorPort = 8;

    public static final int i_kFrontLeftTurningMotorPort = 1;
    public static final int i_kRearLeftTurningMotorPort = 5;
    public static final int i_kFrontRightTurningMotorPort = 3;
    public static final int i_kRearRightTurningMotorPort = 7;

    public static final int i_kFrontLeftEncoderCan = 1;
    public static final int i_kRearLeftEncoderCan = 3;
    public static final int i_kFrontRightEncoderCan = 2;
    public static final int i_kRearRightEncoderCan = 4;

    public static final int pigeonCan = 1;

    public static final String kCANivoreName = "DriveSubsystem";

    public static final double kTrackWidth = util.inchesToMeters(23.75 - 2);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = util.inchesToMeters(25.75 - 2);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    public static final double ksVolts = 0.798;
    public static final double kvVoltSecondsPerMeter = 2.35;
    public static final double kaVoltSecondsSquaredPerMeter = 0.30;

    public static final SimpleMotorFeedforward driveTrainFeedforward = new SimpleMotorFeedforward(
        ksVolts,
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter);

    public static final double kMaxSpeedMetersPerSecond = util.feetToMeters(12.0);
    public static final double i_kMaxSpeedMetersPerSecond = util.feetToMeters(16.3);
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = util.inchesToMeters(4.0);
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * (1.0 / (60.0 / 15.0) / (20.0 / 24.0) / (40.0 / 16.0));

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 0.5;

    public static final double kPModuleDriveController = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class MK4IModuleConstants {
    public static final double i_kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double i_kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final double i_integratedEncoderTicksPerModRev = 2048 * (150.0 / 7.0);

    public static final int i_kEncoderCPR = 4096;
    public static final double i_kWheelDiameterMeters = util.inchesToMeters(4.0);
    public static final double i_kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (i_kWheelDiameterMeters * Math.PI) * (1.0 / (50.0 / 14.0) / (17.0 / 27.0) / (45.0 / 15.0)) / 2048.0;

    public static final double i_kDriveEncoderCountsPerWheelRev = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
        * 2048.0;


    public static final double i_kPModuleTurningController = 0.2;

    public static final double i_kPModuleDriveController = 0.13367;

    public static final double i_kEncoderCountsPerModuleRev = (150.0 / 7.0) * 2048.0;

    public static final double i_kNominalVoltage = 12.0;

    public static final double i_kTurningMotorAllowableClosedLoopError = 10.0;

    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 16 * Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 256 * Math.PI;
    public static final double kModuleMaxSpeedTurningPulsesPer100Ms = kModuleMaxSpeedTurningRadiansPerSecond
        * i_kEncoderCountsPerModuleRev * 0.1;
    public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared = kModuleMaxAccelerationTurningRadiansPerSecondSquared
        * i_kEncoderCountsPerModuleRev * 0.01;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = util.feetToMeters(16.3);
    public static final double kMaxAccelerationMetersPerSecondSquared = util.feetToMeters(16.3);

    public static final double kPXController = 2.0;
    public static final double kPYController = kPXController;
    public static final double kPThetaController = 3.5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = kMaxSpeedMetersPerSecond /
        Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0);
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final ProfiledPIDController AUTON_THETA_CONTROLLER = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    public static final TrajectoryConfig AutonTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
    public static final PIDController AUTON_X_CONTROLLER = new PIDController(AutoConstants.kPXController, 0, 0);
    public static final PIDController AUTON_Y_CONTROLLER = new PIDController(AutoConstants.kPYController, 0, 0);
  }

  public static final class SubsystemConstants {
    /*************** CAN IDS **************/

    // 0-8 will be alotted for swerve motors
    // 9-12 will be alotted and encoders

    public static final int INFEED_MOTOR_ID = 13;
    public static final int SINGULATOR_MOTOR_ID = 14;
    public static final int CONVEYOR_MOTOR_ID = 15;
    public static final int KICKER_MOTOR_ID = 16;
    public static final int SHOOTER_FRONT_MOTOR_ID = 17;
    public static final int SHOOTER_BACK_MOTOR_ID = 18;
    public static final int ANGLE_MOTOR_ID = 16;

    // public static final int CLIMB_MOTOR_ID = 19; // this might need two motors

    public static final int TOF1_SENSOR_ID = 420;
    public static final int TOF2_SENSOR_ID = 69;
  }

  public static final class VBusConstants {
    public static final double kConveyAll = 0.5; // op start
    public static final double kConveyOne = 0.85; // op b
    public static final double kConveyTwo = 0.5; // op a

    public static final double kInfeed = 0.6; // op y
    public static final double kSingulator = 0.5; // op y
    public static final double kInfeedUp = 0.5;
    public static final double kInfeedDown = -0.75;

    public static final double kKicker = 0.5;
    public static final double kAngle = 0.1;

    public static final double kShooterFrontDefault = 0.35;
    public static final double kShooterBackDefault = 0.55;
    public static final double kShooterHoodAngleRotDefault = 14.0;
    // 17 IS ABSOLUTE MAX
  }

  public static final class EncoderConstants {
    public static final double kConveyOne = 20;
    public static final double kConveyTwo = 50;

    public static final double kAngleThreshold = 0.1;
  }

  public static final class IndexConstants {
    public static final double kFineAdjustment = 0.5;
    public static final double kCoarseAdjustment = 1.;

    public static final double kIndexDefault = 14.;
  }

  public static final class LimelightConstants {
    public static final double kTargetHeight = 88.; // 104 when
    public static final double kMountHeight = 21.5; // Might be something like 22. Hard to measure
    public static final double kMountAngle = 46; // is 50 meta?
  }

  public static final class VisionConstants {
    public static final String kCamera1Url = "http://10.40.28.59:1181/stream.mjpg";
    public static final String kCamera2Url = "http://10.40.28.59:1183/stream.mjpg";
  }

  public static final class PIDConstants {
    // TODO: more damping
    public static final class Front {
      public static double kF = 0.05;
      public static double kP = 0.4;
      public static double kD = 0.002;
      public static double kMax = 22000; //20400;
    }

    public static final class Back {
      public static double kF = 0.055;
      public static double kP = 0.1;
      public static double kD = 0.002;
      public static double kMax = 22000; // 17000;
    }

    public static final class Angle {
      public static double kP = 0.1;
    }
  }
}

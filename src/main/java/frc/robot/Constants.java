// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final boolean MK4I = true;
    public static final boolean isNAVX = true;
    public static final boolean isSDS = true;

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


    public static final double kTrackWidth = util.inchesToMeters(23.75);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = util.inchesToMeters(25.75);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false; //true for mk2 chassis

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;

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
  }

  public static final class MK4IModuleConstants {
    public static final double i_kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double i_kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int i_kEncoderCPR = 4096;
    public static final double i_kWheelDiameterMeters = util.inchesToMeters(4.0);
    public static final double i_kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (i_kWheelDiameterMeters * Math.PI) * (1.0 / (50.0 / 14.0) / (17.0 / 27.0) / (45.0 / 15.0));

    public static final double i_kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) i_kEncoderCPR;

    public static final double i_kPModuleTurningController = 0.2;

    public static final double i_kPModuleDriveController = 0;

    public static final double i_kEncoderCountsPerModuleRev = (150/7) * 2048;

    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 16*Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 256*Math.PI;
    public static final double kModuleMaxSpeedTurningPulsesPer100Ms = kModuleMaxSpeedTurningRadiansPerSecond * i_kEncoderCountsPerModuleRev * 0.1;
    public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared = kModuleMaxAccelerationTurningRadiansPerSecondSquared * i_kEncoderCountsPerModuleRev * 0.01;
  }


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = util.feetToMeters(6);
    public static final double kMaxAccelerationMetersPerSecondSquared = util.feetToMeters(4);
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0;
    public static final double kPYController = 0;
    public static final double kPThetaController = 0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}

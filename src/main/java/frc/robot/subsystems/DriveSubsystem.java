// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NavigableMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(146.7);//0.0);//60.6);//60.7
  private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(93.0);//141.2);//139.1
  private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(314.6);//60.6);//60.7
  private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(70.9);//60.3);//60.7


  private static final double i_FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(204.6);//0.0);//60.6);//60.7
  private static final double i_FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(204.2);//141.2);//139.1
  private static final double i_BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(329.4);//60.6);//60.7
  private static final double i_BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(160.6);//60.3);//60.7

  private static DriveSubsystem _instance;
  public static final DriveSubsystem get_instance(){
    if (_instance == null){
      _instance = new DriveSubsystem();
    }
    return _instance;
  }

    //   private final SwerveModuleCAN m_frontLeft =
    //   new SwerveModuleCAN(
    //       i_kFrontLeftDriveMotorPort,
    //       i_kFrontLeftTurningMotorPort,
    //       i_kFrontLeftEncoderCan,
    //       i_FRONT_LEFT_ANGLE_OFFSET);

    //  private final SwerveModuleCAN m_rearLeft =
    //   new SwerveModuleCAN(
    //       i_kRearLeftDriveMotorPort,
    //       i_kRearLeftTurningMotorPort,
    //       i_kRearLeftEncoderCan,
    //       i_BACK_LEFT_ANGLE_OFFSET);

    //  private final SwerveModuleCAN m_frontRight =
    //   new SwerveModuleCAN(
    //       i_kFrontRightDriveMotorPort,
    //       i_kFrontRightTurningMotorPort,
    //       i_kFrontRightEncoderCan,
    //       i_FRONT_RIGHT_ANGLE_OFFSET);

    // private final SwerveModuleCAN m_rearRight =
    //   new SwerveModuleCAN(
    //       i_kRearRightDriveMotorPort,
    //       i_kRearRightTurningMotorPort,
    //       i_kRearRightEncoderCan,
    //       i_BACK_RIGHT_ANGLE_OFFSET);

         private final SwerveModule m_frontLeft =
      new SwerveModule(
          kFrontLeftDriveMotorPort,
          kFrontLeftTurningMotorPort,
          0,
          FRONT_LEFT_ANGLE_OFFSET);

     private final SwerveModule m_rearLeft =
      new SwerveModule(
          kRearLeftDriveMotorPort,
          kRearLeftTurningMotorPort,
          2,
          BACK_LEFT_ANGLE_OFFSET);

     private final SwerveModule m_frontRight =
      new SwerveModule(
          kFrontRightDriveMotorPort,
          kFrontRightTurningMotorPort,
          1,
          FRONT_RIGHT_ANGLE_OFFSET);

    private final SwerveModule m_rearRight =
      new SwerveModule(
          kRearRightDriveMotorPort,
          kRearRightTurningMotorPort,
          3,
          BACK_RIGHT_ANGLE_OFFSET);

  // Robot swerve modules

  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());

    SmartDashboard.putNumber("X (Feet)", util.metersToFeet(m_odometry.getPoseMeters().getX()));
    SmartDashboard.putNumber("Y (Feet)", util.metersToFeet(m_odometry.getPoseMeters().getY()));
    SmartDashboard.putNumber("FL Angle", m_frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("FR Angle", m_frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("RL Angle", m_rearLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("RR Angle", m_rearRight.getState().angle.getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees((kGyroReversed ? -1.0 : 1.0) * m_gyro.getRotation2d().getDegrees()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    //System.out.println(swerveModuleStates[0].speedMetersPerSecond);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }
}

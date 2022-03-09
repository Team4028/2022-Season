// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.i_kFrontLeftDriveMotorPort;
import static frc.robot.Constants.DriveConstants.i_kFrontLeftEncoderCan;
import static frc.robot.Constants.DriveConstants.i_kFrontLeftTurningMotorPort;
import static frc.robot.Constants.DriveConstants.i_kFrontRightDriveMotorPort;
import static frc.robot.Constants.DriveConstants.i_kFrontRightEncoderCan;
import static frc.robot.Constants.DriveConstants.i_kFrontRightTurningMotorPort;
import static frc.robot.Constants.DriveConstants.i_kRearLeftDriveMotorPort;
import static frc.robot.Constants.DriveConstants.i_kRearLeftEncoderCan;
import static frc.robot.Constants.DriveConstants.i_kRearLeftTurningMotorPort;
import static frc.robot.Constants.DriveConstants.i_kRearRightDriveMotorPort;
import static frc.robot.Constants.DriveConstants.i_kRearRightEncoderCan;
import static frc.robot.Constants.DriveConstants.i_kRearRightTurningMotorPort;
import static frc.robot.Constants.DriveConstants.kDriveKinematics;
import static frc.robot.Constants.DriveConstants.kGyroReversed;
import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.util;

public class DriveSubsystem extends SubsystemBase {
  private static final double i_FRONT_LEFT_ANGLE_OFFSET = Math.toRadians(154.6);
  private static final double i_FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(169.3);
  private static final double i_BACK_LEFT_ANGLE_OFFSET = Math.toRadians(31.3);
  private static final double i_BACK_RIGHT_ANGLE_OFFSET = Math.toRadians(199.1);

  private int holdAngleCounter = 0;
  private double holdAngle;
  private boolean enableHoldAngle;

  private static DriveSubsystem _instance;
  public static final DriveSubsystem get_instance(){
    if (_instance == null){
      _instance = new DriveSubsystem();
    }
    return _instance;
  }

      private final SwerveModuleCANTwoElectricBoogaloo m_frontLeft =
      new SwerveModuleCANTwoElectricBoogaloo(
          i_kFrontLeftDriveMotorPort,
          i_kFrontLeftTurningMotorPort,
          i_kFrontLeftEncoderCan,
          i_FRONT_LEFT_ANGLE_OFFSET);

     private final SwerveModuleCANTwoElectricBoogaloo m_rearLeft =
      new SwerveModuleCANTwoElectricBoogaloo(
          i_kRearLeftDriveMotorPort,
          i_kRearLeftTurningMotorPort,
          i_kRearLeftEncoderCan,
          i_BACK_LEFT_ANGLE_OFFSET);

     private final SwerveModuleCANTwoElectricBoogaloo m_frontRight =
      new SwerveModuleCANTwoElectricBoogaloo(
          i_kFrontRightDriveMotorPort,
          i_kFrontRightTurningMotorPort,
          i_kFrontRightEncoderCan,
          i_FRONT_RIGHT_ANGLE_OFFSET);

    private final SwerveModuleCANTwoElectricBoogaloo m_rearRight =
      new SwerveModuleCANTwoElectricBoogaloo(
          i_kRearRightDriveMotorPort,
          i_kRearRightTurningMotorPort,
          i_kRearRightEncoderCan,
          i_BACK_RIGHT_ANGLE_OFFSET);

  // The gyro sensor
  // private final Pigeon2 m_pigeon = new Pigeon2(1);
  private final Gyro m_gyro = DriveConstants.isNAVX? new AHRS(SPI.Port.kMXP): new WPI_Pigeon2(DriveConstants.pigeonCan);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    zeroHeading();
  }

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
    SmartDashboard.putNumber("X (Metres)", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y (Metres)", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Heading (Deg)", m_odometry.getPoseMeters().getRotation().getDegrees());
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
    holdAngleCounter = 0;
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
    double speedScale = DriveConstants.BASE_SPEED_SCALE + RobotContainer.get_instance().getRightTrigger() * (1 - DriveConstants.BASE_SPEED_SCALE);
    xSpeed *= speedScale * DriveConstants.i_kMaxSpeedMetersPerSecond;
    ySpeed *= speedScale * DriveConstants.i_kMaxSpeedMetersPerSecond;
    rot *= speedScale * DriveConstants.i_kMaxSpeedMetersPerSecond;
    if (rot == 0){
      if (holdAngleCounter < 1 && enableHoldAngle){
        holdAngleCounter++;
        holdAngle = m_gyro.getRotation2d().getRadians();
      }
      rot = AutoConstants.AUTO_THETA_CONTROLLER.calculate(m_gyro.getRotation2d().getRadians(), holdAngle);
    }
    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees((kGyroReversed ? -1.0 : 1.0) * m_gyro.getRotation2d().getDegrees()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.i_kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
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

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    resetOdometry(new Pose2d(0, 0, m_gyro.getRotation2d()));
  }

  public void setEnableHoldAngle(boolean enable){
    enableHoldAngle = enable;
  }
  public void toggleEnableHoldAngle(){
    enableHoldAngle = !enableHoldAngle;
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

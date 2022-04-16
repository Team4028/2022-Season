// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  //COMPETITION CHASSIS VALUES
  private static final double i_FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(139.5);//139.6);//206.3);//24.32 + 180.0);//154.6);
  private static final double i_FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(322.3);//144.4);//336.0 - 180.0);//169.3 - 5);
  private static final double i_BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(105.7);//107);//327.7);//507.2 - 180.0);//31.3);
  private static final double i_BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(233.9);//234.5);//160.9);//340.1 - 180.0);//199.1);

  private int testTimer = 0;
  private int configWaitCycles = 50;
  private Rotation2d zeroAbsoluteCompassHeading;

  private static DriveSubsystem _instance;

  private int updateCycles = 0;

  public static final DriveSubsystem getInstance() {
    if (_instance == null) {
      _instance = new DriveSubsystem();
    }
    return _instance;
  }

  public final SwerveModuleCANTwoElectricBoogaloo m_frontLeft = new SwerveModuleCANTwoElectricBoogaloo(
      i_kFrontLeftDriveMotorPort,
      i_kFrontLeftTurningMotorPort,
      i_kFrontLeftEncoderCan,
      i_FRONT_LEFT_ANGLE_OFFSET);

  public final SwerveModuleCANTwoElectricBoogaloo m_rearLeft = new SwerveModuleCANTwoElectricBoogaloo(
      i_kRearLeftDriveMotorPort,
      i_kRearLeftTurningMotorPort,
      i_kRearLeftEncoderCan,
      i_BACK_LEFT_ANGLE_OFFSET);

  public final SwerveModuleCANTwoElectricBoogaloo m_frontRight = new SwerveModuleCANTwoElectricBoogaloo(
      i_kFrontRightDriveMotorPort,
      i_kFrontRightTurningMotorPort,
      i_kFrontRightEncoderCan,
      i_FRONT_RIGHT_ANGLE_OFFSET);

  public final SwerveModuleCANTwoElectricBoogaloo m_rearRight = new SwerveModuleCANTwoElectricBoogaloo(
      i_kRearRightDriveMotorPort,
      i_kRearRightTurningMotorPort,
      i_kRearRightEncoderCan,
      i_BACK_RIGHT_ANGLE_OFFSET);

  // The gyro sensor
  // private final Pigeon2 m_pigeon = new Pigeon2(1);
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(DriveConstants.pigeonCan, DriveConstants.kCANivoreName);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kDriveKinematics, getGyroRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //zeroHeading();
    //m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 50);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block -- only during auton/disabled
    // Not needed during teleop
    if (/*!DriverStation.isTeleopEnabled() && */testTimer > 8 * configWaitCycles) {
      m_odometry.update(
          getGyroRotation2d(),
          m_frontLeft.getState(),
          m_rearLeft.getState(),
          m_frontRight.getState(),
          m_rearRight.getState());
    }

    //TODO: Organized, comprehensive data for whole Drivetrain
    if (updateCycles == 3) {
      // SmartDashboard.putNumber("X (Feet)", Units.metersToFeet(m_odometry.getPoseMeters().getX()));
      // SmartDashboard.putNumber("Y (Feet)", Units.metersToFeet(m_odometry.getPoseMeters().getY()));
      SmartDashboard.putNumber("X (Metres)", m_odometry.getPoseMeters().getX());
      SmartDashboard.putNumber("Y (Metres)", m_odometry.getPoseMeters().getY());
      SmartDashboard.putNumber("Heading (Deg)", m_odometry.getPoseMeters().getRotation().getDegrees());
    //   SmartDashboard.putNumber("FL Angle", m_frontLeft.getState().angle.getDegrees());
    //   SmartDashboard.putNumber("FR Angle", m_frontRight.getState().angle.getDegrees());
    //   SmartDashboard.putNumber("RL Angle", m_rearLeft.getState().angle.getDegrees());
    //   SmartDashboard.putNumber("RR Angle", m_rearRight.getState().angle.getDegrees());
      //SmartDashboard.putBoolean("Hold Angle", enableHoldAngle);
      updateCycles = 0;
    } else {
      updateCycles++;
    }

    //TODO: Fix this/ remove if possible
    if(testTimer < 8 * configWaitCycles + 3){
      testTimer++;
    }
    if (testTimer == configWaitCycles) {
      System.out.println("we are worse 1");
      m_frontLeft.configDriveMotor();
      zeroHeading();
    }else if (testTimer == 2 * configWaitCycles){
      System.out.println("we are worse 2");
      m_frontRight.configDriveMotor();
    }else if (testTimer == 3 * configWaitCycles){
      System.out.println("we are worse 3");
      m_rearLeft.configDriveMotor();
    }else if (testTimer == 4 * configWaitCycles){
      System.out.println("we are worse 4");
      m_rearRight.configDriveMotor();
    } else if(testTimer == 5 * configWaitCycles){
      m_frontLeft.configTurningMotor();
    }else if(testTimer == 6 * configWaitCycles){
      m_rearRight.configTurningMotor();
    }else if(testTimer == 7 * configWaitCycles){
      m_frontRight.configTurningMotor();
      m_frontLeft.configDriveMotor();
    }else if(testTimer == 8 * configWaitCycles){
      m_rearLeft.configTurningMotor();
      zeroHeading();
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public Rotation2d getGyroRotation2d(){
    return Rotation2d.fromDegrees((kGyroReversed ? -1.0 : 1.0) * m_gyro.getRotation2d().getDegrees());
  }
  //TODO: Verify experimental odometry resetting
  public Rotation2d getAbsoluteGyro(){
    return Rotation2d.fromDegrees(m_gyro.getAbsoluteCompassHeading()).minus(zeroAbsoluteCompassHeading);
  }
  public void zeroAbsoluteCompassHeading(){
    zeroAbsoluteCompassHeading = Rotation2d.fromDegrees(m_gyro.getAbsoluteCompassHeading()).minus(m_odometry.getPoseMeters().getRotation());
  }
  /**
   * Be sure Limelight is aligned fairly well, robot is not moving
   * If using Limelight distance used for shooter tables, be sure to SUBTRACT 14.5 Inches from that distance
   * @param visionDistanceMeters Distance from center of goal to center of robot
   */
  public void resetOdometryWithVision(double visionDistanceMeters){
    Pose2d visionCurrentPose = new Pose2d(new Translation2d(Units.inchesToMeters(324.0), Units.inchesToMeters(162.0))
    .minus(new Translation2d(visionDistanceMeters, getAbsoluteGyro())), getGyroRotation2d());
    m_odometry.resetPosition(visionCurrentPose, getGyroRotation2d());
  }
  //////

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getGyroRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, BooleanSupplier fieldRelative) {
    xSpeed *= DriveConstants.i_kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.i_kMaxSpeedMetersPerSecond;
    rot *= AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative.getAsBoolean()
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_odometry.getPoseMeters().getRotation().getDegrees()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, i_kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, i_kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }


  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }
  private void resetModuleHeadingControllers(){
    m_frontLeft.RezeroTurningMotorEncoder();
    m_frontRight.RezeroTurningMotorEncoder();
    m_rearLeft.RezeroTurningMotorEncoder();
    m_rearRight.RezeroTurningMotorEncoder();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    resetModuleHeadingControllers();
    m_odometry.resetPosition(new Pose2d(), getGyroRotation2d());
  }

  public double getDriveXVelocity(){
    return kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    ).vxMetersPerSecond;
  }
  public double getDriveYVelocity(){
    return kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    ).vxMetersPerSecond;
  }
  public double getDriveAngularVelocity(){
    return kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState())
      .omegaRadiansPerSecond;
  }
  public Rotation2d getOdometryAngleToTarget(){
    return new Rotation2d(Math.atan2(Units.inchesToMeters(324.0) - m_odometry.getPoseMeters().getX(), Units.inchesToMeters(162.0) - m_odometry.getPoseMeters().getY()));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation2d().getDegrees();
  }

}

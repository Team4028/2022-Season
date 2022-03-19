// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  private static final double i_FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(206.3);// 24.32 + 180.0);//154.6);
  private static final double i_FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(144.4 - 9.0);// 336.0 - 180.0);//169.3 - 5);
  private static final double i_BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(327.7);// 507.2 - 180.0);//31.3);
  private static final double i_BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(160.9);// 340.1 - 180.0);//199.1);

  private int holdAngleCounter = 0;
  private double holdAngle;
  private boolean enableHoldAngle = false;
  private int testTimer = 0;
  private int configWaitCycles = 50;
  private WheelSpeeds _WheelSpeeds = new WheelSpeeds();

  private static DriveSubsystem _instance;

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
    // Update the odometry in the periodic block
    if (testTimer > 8 * configWaitCycles) {
      m_odometry.update(
          getGyroRotation2d(),
          m_frontLeft.getState(),
          m_rearLeft.getState(),
          m_frontRight.getState(),
          m_rearRight.getState());
    }
    //TODO: Organized, comprehensive data for whole Drivetrain
    SmartDashboard.putNumber("X (Feet)", Units.metersToFeet(m_odometry.getPoseMeters().getX()));
    SmartDashboard.putNumber("Y (Feet)", Units.metersToFeet(m_odometry.getPoseMeters().getY()));
    SmartDashboard.putNumber("X (Metres)", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y (Metres)", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Heading (Deg)", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("FL Angle", m_frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("FR Angle", m_frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("RL Angle", m_rearLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("RR Angle", m_rearRight.getState().angle.getDegrees());
    SmartDashboard.putBoolean("Hold Angle", enableHoldAngle);

    //TODO: Fix this/ remove if possible
    if(testTimer < 8 * configWaitCycles + 1){
      testTimer++;
    }
    if (testTimer == configWaitCycles) {
      System.out.println("we are worse 1");
      DriveSubsystem.getInstance().m_frontLeft.configDriveMotor();
      zeroHeading();
    }else if (testTimer == 2 * configWaitCycles){
      System.out.println("we are worse 2");
      DriveSubsystem.getInstance().m_frontRight.configDriveMotor();
    }else if (testTimer == 3 * configWaitCycles){
      System.out.println("we are worse 3");
      DriveSubsystem.getInstance().m_rearLeft.configDriveMotor();
    }else if (testTimer == 4 * configWaitCycles){
      System.out.println("we are worse 4");
      DriveSubsystem.getInstance().m_rearRight.configDriveMotor();
    } else if(testTimer == 5 * configWaitCycles){
      DriveSubsystem.getInstance().m_frontLeft.configTurningMotor();
    }else if(testTimer == 6 * configWaitCycles){
      DriveSubsystem.getInstance().m_rearRight.configTurningMotor();
    }else if(testTimer == 7 * configWaitCycles){
      DriveSubsystem.getInstance().m_frontRight.configTurningMotor();
    }else if(testTimer == 8 * configWaitCycles){
      DriveSubsystem.getInstance().m_rearLeft.configTurningMotor();
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

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getGyroRotation2d());
    holdAngleCounter = 0;
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double speedScale = DriveConstants.BASE_SPEED_SCALE
        + RobotContainer.getInstance().getRightTrigger() * (1 - DriveConstants.BASE_SPEED_SCALE);
    xSpeed *= speedScale * DriveConstants.i_kMaxSpeedMetersPerSecond;
    ySpeed *= speedScale * DriveConstants.i_kMaxSpeedMetersPerSecond;
    rot *= speedScale * DriveConstants.i_kMaxSpeedMetersPerSecond;
    if (rot == 0 && enableHoldAngle) {
      if (holdAngleCounter < 1) {
        holdAngleCounter++;
        holdAngle = getGyroRotation2d().getRadians();
      }
      rot = AutoConstants.AUTON_THETA_CONTROLLER.calculate(getGyroRotation2d().getRadians(), holdAngle);
    }
    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_odometry.getPoseMeters().getRotation().getDegrees()))
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
        desiredStates, i_kMaxSpeedMetersPerSecond);
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
  private void resetModuleHeadingControllers(){
    DriveSubsystem.getInstance().m_frontLeft.RezeroTurningMotorEncoder();
    DriveSubsystem.getInstance().m_frontRight.RezeroTurningMotorEncoder();
    DriveSubsystem.getInstance().m_rearLeft.RezeroTurningMotorEncoder();
    DriveSubsystem.getInstance().m_rearRight.RezeroTurningMotorEncoder();
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

  public void setEnableHoldAngle(boolean enable) {
    enableHoldAngle = enable;
  }

  public void toggleEnableHoldAngle() {
    enableHoldAngle = !enableHoldAngle;
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

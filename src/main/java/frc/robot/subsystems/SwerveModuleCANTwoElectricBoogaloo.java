// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MK4IModuleConstants;

public class SwerveModuleCANTwoElectricBoogaloo {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private final WPI_CANCoder m_turningEncoder;

  private final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private final int CAN_TIMEOUT_MS = 250;

  private TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

  private int resetIterations = 0;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleCANTwoElectricBoogaloo(
      int driveMotorChannel,
      int turningMotorChannel,
      int CANEncoderPort,
      double turningMotorOffset) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    m_turningEncoder = new WPI_CANCoder(CANEncoderPort);

    m_turningEncoder.configFactoryDefault();
    m_turningMotor.configFactoryDefault();
    m_driveMotor.configFactoryDefault();
    m_turningEncoder.setPositionToAbsolute();
    if(m_turningEncoder.configGetSensorInitializationStrategy() != SensorInitializationStrategy.BootToAbsolutePosition){
        m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, CAN_TIMEOUT_MS);
    }
    m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS);
    
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    m_turningMotor.configSelectedFeedbackCoefficient(1);
    m_turningEncoder.configMagnetOffset(Math.toDegrees(turningMotorOffset));
    m_turningMotor.setSensorPhase(true);
    m_turningMotor.configAllowableClosedloopError(0, 10, CAN_TIMEOUT_MS);
    m_turningMotor.setStatusFramePeriod(
              StatusFrameEnhanced.Status_1_General,
              STATUS_FRAME_GENERAL_PERIOD_MS,
              CAN_TIMEOUT_MS
      );

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.configSelectedFeedbackCoefficient(1);
    m_turningMotor.setNeutralMode(NeutralMode.Coast);
    m_turningMotor.setInverted(true);
    m_turningMotor.selectProfileSlot(0, 0);
    m_turningMotor.setSelectedSensorPosition(m_turningEncoder.getPosition() / 360 * 2048 * 150 / 7);
    m_turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 1.0));
    

    configMotorPID(m_turningMotor, 0, 0.2, 0.0, 0.1);
    System.out.println(m_turningMotor.getSelectedSensorPosition());
  }

  private double getTurningEncoderRadians(){
    return Math.toRadians(m_turningMotor.getSelectedSensorPosition(0) * (7/150) * Math.PI * 2 / 2048);
    }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity() * MK4IModuleConstants.i_kDriveEncoderDistancePerPulse * 10, new Rotation2d(getTurningEncoderRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    checkMotorCoder();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        optimize(SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians())), getTurningEncoderRadians());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        DriveConstants.driveTrainFeedforward.calculate(state.speedMetersPerSecond);

    m_driveMotor.setVoltage(driveOutput);
    m_turningMotor.set(ControlMode.Position, getTurnPulses(state.angle.getRadians()));
  }

  public void configMotorPID(WPI_TalonFX talon, int slotIdx, double p, double i, double d){
    talon.config_kP(slotIdx, p, CAN_TIMEOUT_MS);
    talon.config_kI(slotIdx, i, CAN_TIMEOUT_MS);
    talon.config_kD(slotIdx, d, CAN_TIMEOUT_MS);
  }

//Zeros all the SwerveModule encoders.
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }

private SwerveModuleState optimize(SwerveModuleState st, double currentAngleRadians){
  double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
  if (currentAngleRadiansMod < 0.0) {
      currentAngleRadiansMod += 2.0 * Math.PI;
  }

  double adjustedReferenceAngleRadians = st.angle.getRadians() + currentAngleRadians - currentAngleRadiansMod;
  if (st.angle.getRadians() - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
  } else if (st.angle.getRadians() - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
  }

  st = new SwerveModuleState(st.speedMetersPerSecond, new Rotation2d(adjustedReferenceAngleRadians));
  double changeAngleRads = st.angle.getRadians() - currentAngleRadians;
  while(Math.abs(changeAngleRads) > Math.PI / 2){
  if(changeAngleRads > Math.PI / 2){
    st = new SwerveModuleState(-st.speedMetersPerSecond, new Rotation2d(st.angle.getRadians() - Math.PI));
  } else if(changeAngleRads < -Math.PI / 2){
    st = new SwerveModuleState(-st.speedMetersPerSecond, new Rotation2d(st.angle.getRadians() + Math.PI));
  }
  changeAngleRads = st.angle.getRadians() - currentAngleRadians;
}

  return st;
}
private double getoptimizedchange(SwerveModuleState st, double currentAngleRadians){
double changeAngleRads = st.angle.getRadians() - currentAngleRadians;
while(Math.abs(changeAngleRads) > Math.PI / 2){
if(changeAngleRads > Math.PI / 2){
  st = new SwerveModuleState(-st.speedMetersPerSecond, new Rotation2d(st.angle.getRadians() - Math.PI));
} else if(changeAngleRads < -Math.PI / 2){
  st = new SwerveModuleState(-st.speedMetersPerSecond, new Rotation2d(st.angle.getRadians() + Math.PI));
}
changeAngleRads = st.angle.getRadians() - currentAngleRadians;
}
return changeAngleRads;
}
private double getTurnPulses(double referenceAngleRadians){
  return referenceAngleRadians * MK4IModuleConstants.i_kEncoderCountsPerModuleRev / 2 / Math.PI;
}

private void checkMotorCoder(){
  if(resetIterations <1){
    m_turningMotor.setSelectedSensorPosition(m_turningEncoder.getAbsolutePosition() / 360 * 2048 * (150/7));
  }
  resetIterations++;
}

}

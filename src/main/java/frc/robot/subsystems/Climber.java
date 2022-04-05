// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.RampRateConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VBusConstants;

public class Climber extends SubsystemBase {
  private static Climber _instance = new Climber();

  private Solenoid _tippy;
  private Solenoid _grippy;

  private CANSparkMax _left;
  private CANSparkMax _right;

  private RelativeEncoder _leftEncoder;
  private RelativeEncoder _rightEncoder;

  /** Creates a new cry about it. */
  public Climber() {
    _tippy = new Solenoid(PneumaticsModuleType.CTREPCM, SubsystemConstants.TIPPY_SOLENOID_ID);
    _grippy = new Solenoid(PneumaticsModuleType.CTREPCM, SubsystemConstants.GRIPPY_SOLENOID_ID);

    _left = new CANSparkMax(SubsystemConstants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
    _right = new CANSparkMax(SubsystemConstants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    _left.restoreFactoryDefaults();
    _right.restoreFactoryDefaults();

    _left.setSmartCurrentLimit(CurrentLimitConstants.kClimber);
    _right.setSmartCurrentLimit(CurrentLimitConstants.kClimber);

    _left.setIdleMode(IdleMode.kBrake);
    _right.setIdleMode(IdleMode.kBrake);

    _left.setOpenLoopRampRate(RampRateConstants.kClimber);
    _right.setOpenLoopRampRate(RampRateConstants.kClimber);

    _leftEncoder = _left.getEncoder();
    _rightEncoder = _right.getEncoder();

    resetLeftEncoder();
    resetRightEncoder();
  }

  public void toggleTippySolenoid() {
    _tippy.set(!_tippy.get());
  }

  public void toggleGrippySolenoid() {
    _grippy.set(!_grippy.get());
  }

  public double getLeftEncoderPosition() {
    return _leftEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return _rightEncoder.getPosition();
  }

  public void resetLeftEncoder() {
    _leftEncoder.setPosition(0.);
  }

  public void resetRightEncoder() {
    _rightEncoder.setPosition(0.);
  }

  public void leftMotorForward(double speed) {
    _left.set(speed);
  }

  public void leftMotorBackward(double speed) {
    _left.set(speed);
  }

  public void rightMotorForward(double speed) {
    _right.set(speed);
  }

  public void rightMotorBackward(double speed) {
    _right.set(speed);
  }

  public void slowDrop() {
    _right.set(-VBusConstants.kClimberSlow);
    _left.set(-VBusConstants.kClimberSlow);
  }

  public void slowUp() {
    _right.set(VBusConstants.kClimberSlow);
    _left.set(VBusConstants.kClimberSlow);
  }

  public void leftMotorOff() {
    _left.set(0.);
  }

  public void rightMotorOff() {
    _right.set(0.);
  }

  public void resetEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  public void setRightEncoder(double val) {
    _rightEncoder.setPosition(val);
  }

  public void setLeftEncoder(double val) {
    _leftEncoder.setPosition(val);
  }

  public void stop() {
    rightMotorOff();
    leftMotorOff();
  }

  public static Climber getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber", _leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber", _rightEncoder.getPosition());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VBusConstants;

public class TestingEther extends SubsystemBase {
  /** Creates a new TestingEther. */
  private TalonSRX _infeedMotor;
  private CANSparkMax _singulatorMotor;
  private CANSparkMax _conveyorMotor; // needs to be reversed

  // private TalonFX _shooterMotorOne;
  // private TalonFX _shooterMotorTwo;
  private boolean isTargetReached = false;

  private RelativeEncoder _enc;
  private static TestingEther _instance = new TestingEther();
 public static TestingEther get_instance() {
   return _instance;
 }


  public TestingEther() {
  _infeedMotor = new TalonSRX(SubsystemConstants.INFEED_MOTOR_ID);
  _singulatorMotor = new CANSparkMax(SubsystemConstants.SINGULATOR_MOTOR_ID, MotorType.kBrushless);
  _conveyorMotor = new CANSparkMax(SubsystemConstants.CONVEYOR_MOTOR_ID, MotorType.kBrushless);

  // _shooterMotorOne = new TalonFX(SubsystemConstants.SHOOTER_1_MOTOR_ID);
  // _shooterMotorTwo = new TalonFX(SubsystemConstants.SHOOTER_2_MOTOR_ID);

  // _shooterMotorTwo.setInverted(InvertType.InvertMotorOutput);

    _enc = _conveyorMotor.getEncoder();
    _enc.setPosition(0);

    _conveyorMotor.setInverted(true);
 
  }

  public void runInfeedSingulatorMotors(double mult){
    _infeedMotor.set(ControlMode.PercentOutput, mult * VBusConstants.kInfeed);
    _singulatorMotor.set(mult * VBusConstants.kSingulator);
  }

  public void runConveyorMotor(double vbus){
    _conveyorMotor.set(vbus);
  }

  public void stopConveyorMotor(){
    _conveyorMotor.set(0);
  }

  public void stopInfeedSingulatorConveyorMotors(){
    _infeedMotor.set(ControlMode.PercentOutput, 0);
    _singulatorMotor.set(0);
    _conveyorMotor.set(0);
  }

  public void runConveyorMotorWithEncoder(double target, double vbus){
    isTargetReached = false;
    if (_enc.getPosition()>target)
    {
      _conveyorMotor.set(0);
      isTargetReached = true;
    }
    else{
      _conveyorMotor.set(vbus);
    }
    
  }

  

  

  // public void runShooterMotors(){
  //   _shooterMotorOne.set(ControlMode.PercentOutput, .47);
  //   _shooterMotorTwo.set(ControlMode.PercentOutput, .67);
  // }

  // public void stopShooterMotors(){
  //   _shooterMotorOne.set(ControlMode.PercentOutput, 0);
  //   _shooterMotorTwo.set(ControlMode.PercentOutput, 0);
  // }
  public void resetEncoder(){
    _enc.setPosition(0);
  }

  public boolean getIsTargetReached(){
    return isTargetReached;
  }//tempmethod


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

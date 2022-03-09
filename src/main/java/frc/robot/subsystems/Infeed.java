// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VBusConstants;

public class Infeed extends SubsystemBase {
  /** Creates a new SingulatorAndInfeed. */
  private TalonSRX _infeedMotor;
  private CANSparkMax _singulatorMotor;
  private static Infeed _instance = new Infeed();
  public static Infeed get_instance() {
   return _instance;
  }
  public Infeed() {
    _infeedMotor = new TalonSRX(SubsystemConstants.INFEED_MOTOR_ID);
    _singulatorMotor = new CANSparkMax(SubsystemConstants.SINGULATOR_MOTOR_ID, MotorType.kBrushless);
 
  }
  public void runInfeedSingulatorMotors(double mult){
    _infeedMotor.set(ControlMode.PercentOutput, mult * VBusConstants.kInfeed);
    System.out.println("burh");
    _singulatorMotor.set(mult * VBusConstants.kSingulator);
  }
  
  public void stopInfeedSingulatorMotors(){
    _infeedMotor.set(ControlMode.PercentOutput, 0);
    _singulatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

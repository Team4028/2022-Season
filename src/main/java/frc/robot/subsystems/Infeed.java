// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Infeed extends SubsystemBase {
  /** Creates a new Infeed. */
  private TalonSRX _MotorInfeedOne;
  private static Infeed _instance = new Infeed();
  public static Infeed get_instance() {
    return _instance;
  }

  public Infeed() {
    //_MotorInfeedOne = new TalonSRX(Constants.INFEED_MOTOR_ID);
    
    
  }

  public void runMotorInfeed(){
   // _MotorInfeedOne.set(ControlMode.PercentOutput, .1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Singulator extends SubsystemBase {
  /** Creates a new Singulator. */
  //private CANSparkMax _MotorSingulatorOne;
  public static Singulator get_instance() {
    return get_instance();
  }

  public Singulator() {
   // _MotorSingulatorOne = new CANSparkMax(Constants.SINGULATOR_MOTOR_ID, MotorType.kBrushless);
    
  }

  public void runMotorSingulatorOne(){
   // _MotorSingulatorOne.set(.1);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

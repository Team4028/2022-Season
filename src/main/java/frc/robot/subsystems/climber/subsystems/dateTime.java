// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Date;
import java.util.Scanner;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class dateTime extends SubsystemBase {
  long startTime;
  long currentTime;
  long elapsedTime;
  private static dateTime _instance = new dateTime();
  private CANSparkMax _m;

  public dateTime() {
    startTime = new Date().getTime();
    _m = new CANSparkMax(3, MotorType.kBrushless);
  }


  @Override
  public void periodic() {
  }
  public static dateTime get_instance() {
    return _instance;
  }
  public void runWithTime() {
    elapsedTime = new Date().getTime()-startTime;
    System.out.println(elapsedTime);
    if(elapsedTime < 3000) {
     _m.set(.2);
    } else if(3000 < elapsedTime){
     _m.set(-.2);
    } else {
      _m.set(0);
    }
      
    }
    


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

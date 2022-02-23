// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VBusConstants;

public class Motor extends SubsystemBase {
  private TalonFX _front = new TalonFX(17);
  private TalonFX _back = new TalonFX(18);

  public double front_kF, front_kP, front_kI,
                front_kD, front_kIz, front_kAccel,
                front_kMax;

  public double back_kF, back_kP, back_kI,
                back_kD, back_kIz, back_kAccel,
                back_kMax;

  public static class Shot{
    public double speed;
    public double actuatorPosition;

    public Shot(double spd, double actPos){
      speed = spd;
      actuatorPosition = actPos;
    }

    public static Shot zeroShot = new Shot(0, .3);

    //public static Shot getStopShot(){
      //return new Shot(0, getInstance()._linearActuator.get());
    //}
}

  private static Motor _instance = new Motor();
  /** Creates a new Motor. */
  public Motor() {
    _front.configFactoryDefault();
    _back.configFactoryDefault();

    // I'M A LEGEND LETS GOOOOOOOOOOOOO
    front_kF = 0.05;
    back_kF = 0.055;

    front_kP = 0.3;
    back_kP = 0.17;

    front_kI = 0.001;
    back_kI = 0.001;

    front_kD = 5.;
    back_kD = 5.;

    front_kMax = 20400;
    back_kMax = 17000;

    front_kAccel = VBusConstants.kShooterFront * front_kMax;
    back_kAccel = VBusConstants.kShooterBack * back_kMax;

    _front.config_kF(0, front_kF);
    _back.config_kF(0, back_kF);

    _front.config_kP(0, front_kP);
    _back.config_kP(0, back_kP);

    _front.config_kI(0, front_kI);
    _back.config_kI(0, back_kI);

    _front.config_kD(0, front_kD);
    _back.config_kD(0, back_kD);

    SmartDashboard.putNumber("Front Shooter P Gain", front_kP);
    SmartDashboard.putNumber("Front Shooter I Gain", front_kI);
    SmartDashboard.putNumber("Front Shooter D Gain", front_kD);
    SmartDashboard.putNumber("Front Shooter I Zone", front_kIz);
    SmartDashboard.putNumber("Front Shooter Feed Forward", front_kF);
    SmartDashboard.putNumber("Front Shooter VBus", VBusConstants.kShooterFront);
    
    SmartDashboard.putNumber("Back Shooter P Gain", back_kP);
    SmartDashboard.putNumber("Back Shooter I Gain", back_kI);
    SmartDashboard.putNumber("Back Shooter D Gain", back_kD);
    SmartDashboard.putNumber("Back Shooter I Zone", back_kIz);
    SmartDashboard.putNumber("Back Shooter Feed Forward", back_kF);
    SmartDashboard.putNumber("Back Shooter VBus", VBusConstants.kShooterBack);
  }

  public void update() {
    double front_p = SmartDashboard.getNumber("Front Shooter P Gain", 0);
    double front_i = SmartDashboard.getNumber("Front Shooter I Gain", 0);
    double front_d = SmartDashboard.getNumber("Front Shooter D Gain", 0);
    double front_iz = SmartDashboard.getNumber("Front Shooter I Zone", 0);
    double front_f = SmartDashboard.getNumber("Front Shooter Feed Forward", 0);
    double front_vbus = SmartDashboard.getNumber("Front Shooter VBus", 0);

    if((front_p != front_kP)) { _front.config_kP(0, front_p); front_kP = front_p; }
    if((front_i != front_kI)) { _front.config_kI(0, front_i); front_kI = front_i; }
    if((front_d != front_kD)) { _front.config_kD(0, front_d); front_kD = front_d; }
    if((front_iz != front_kIz)) { _front.config_IntegralZone(0, front_iz); front_kIz = front_iz; }
    if((front_f != front_kF)) { _front.config_kF(0, front_f); front_kF = front_f; }
    if((VBusConstants.kShooterFront != front_vbus)) { VBusConstants.kShooterFront = front_vbus; }

    double back_p = SmartDashboard.getNumber("Back Shooter P Gain", 0);
    double back_i = SmartDashboard.getNumber("Back Shooter I Gain", 0);
    double back_d = SmartDashboard.getNumber("Back Shooter D Gain", 0);
    double back_iz = SmartDashboard.getNumber("Back Shooter I Zone", 0);
    double back_f = SmartDashboard.getNumber("Back Shooter Feed Forward", 0);
    double back_vbus = SmartDashboard.getNumber("Back Shooter VBus", 0);

    if((back_p != back_kP)) { _back.config_kP(0, back_p); back_kP = back_p; }
    if((back_i != back_kI)) { _back.config_kI(0, back_i); back_kI = back_i; }
    if((back_d != back_kD)) { _back.config_kD(0, back_d); back_kD = back_d; }
    if((back_iz != back_kIz)) { _back.config_IntegralZone(0, back_iz); back_kIz = back_iz; }
    if((back_f != back_kF)) { _back.config_kF(0, back_f); back_kF = back_f; }
    if((VBusConstants.kShooterBack != back_vbus)) { VBusConstants.kShooterBack = back_vbus; }
  }

  public void run() {
    _front.set(ControlMode.Velocity, front_kAccel);
    _back.set(ControlMode.Velocity, back_kAccel);

    SmartDashboard.putNumber("Front Shooter Error", _front.getClosedLoopError());
    SmartDashboard.putNumber("Back Shooter Error", _back.getClosedLoopError());
  }

  public static Motor getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VBusConstants;

public class Infeed extends SubsystemBase {
  /** Creates a new SingulatorAndInfeed. */
  private TalonSRX _infeedMotor;
  private CANSparkMax _singulatorMotor;
  private Solenoid _solenoid;
  private static Infeed _instance = new Infeed();

  public static Infeed getInstance() {
    return _instance;
  }

  public Infeed() {
    _infeedMotor = new TalonSRX(SubsystemConstants.INFEED_MOTOR_ID);
    _singulatorMotor = new CANSparkMax(SubsystemConstants.SINGULATOR_MOTOR_ID, MotorType.kBrushless);
    _solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SubsystemConstants.INFEED_SOLENOID_ID);
    _infeedMotor.configFactoryDefault();
    _infeedMotor.setInverted(true);
    _singulatorMotor.setInverted(false);

    _infeedMotor.setInverted(true);
  }

  public void runInfeedSingulatorMotors(double mult) {
    if (_solenoid.get()) {
      _infeedMotor.set(ControlMode.PercentOutput, mult * VBusConstants.kInfeed);
      // SmartDashboard.putBoolean("Infeed/Running", true);
      // SmartDashboard.putNumber("Infeed/Vbus", mult * VBusConstants.kInfeed);
    } else {
      _infeedMotor.set(ControlMode.PercentOutput, 0.);
      // SmartDashboard.putBoolean("Infeed/Running", false);
    }
    _singulatorMotor.set(mult * VBusConstants.kSingulator);
  //   SmartDashboard.putBoolean("Singulator/Running", true);
  //   SmartDashboard.putNumber("Singulator/Vbus", mult * VBusConstants.kInfeed);
  }

  public void stopInfeedSingulatorMotors() {
    _infeedMotor.set(ControlMode.PercentOutput, 0);
    _singulatorMotor.set(0);
    // SmartDashboard.putBoolean("Infeed/Running", false);
    // SmartDashboard.putBoolean("Singulator/Running", false);

  }

  public void toggleInfeedUp() {
    _solenoid.toggle();
  }
  public void setInfeedDown(){
    _solenoid.set(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

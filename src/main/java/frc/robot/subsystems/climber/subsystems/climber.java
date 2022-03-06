// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climber extends SubsystemBase {
  /** Creates a new motor. */
  CANSparkMax _grip1;
  CANSparkMax _grip2;
  DigitalInput _button;
  RelativeEncoder _encoderGrip;
  DoubleSolenoid _gripSol1;
  DoubleSolenoid _gripSol2;
  DoubleSolenoid _tipSol1;
  DoubleSolenoid _tipSol2;
  private static climber _instance = new climber();
  public climber() {
    
    _grip1 = new CANSparkMax(1, MotorType.kBrushless);
    _grip2 = new CANSparkMax(2, MotorType.kBrushless);
    _gripSol1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    _gripSol2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    _tipSol1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    _tipSol2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    _button = new DigitalInput(0);
    _encoderGrip = _grip1.getEncoder();
    grippyFollow();
  }
  public void setGrip(double vbus){
    _grip1.set(vbus);
  }
  public void setTipSol1(Value value) {
    _tipSol1.set(value);
  }
  public void setTipSol2(Value value) {
    _tipSol2.set(value);
  }
  public void setGripSol1(Value value) {
    _gripSol1.set(value);
  }
  public void setGripSol2(Value value) {
    _gripSol2.set(value);
  }
  public double getEncoderPos() {
    return _encoderGrip.getPosition();
  }
  public void zeroEncoder() {
    _encoderGrip.setPosition(0.);
  }
  public void grippyFollow() {
    _grip2.follow(_grip1);
  }
  public boolean isClosed(){
    return _button.get();
  }
   public static climber get_instance() {
      return _instance;
   }
    



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

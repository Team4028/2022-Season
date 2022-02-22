// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private static Shooter _instance = new Shooter();
  double limeLightDistance, shooterIndex;
  
  public static Shooter getInstance() {
    return _instance;
  }

  public Shooter() {
    shooterIndex = 6;
  }
  public double index() {
    return shooterIndex;
  }
 /* public double incrementIndexFine() {
    shooterIndex = shooterIndex + 0.5;
    return shooterIndex;*/
  //}
  public double incrementIndex() {
    shooterIndex = shooterIndex + 1;
    return shooterIndex;
  }
 /* public double decrementIndexFine() {
    shooterIndex = shooterIndex - 0.5;
    return shooterIndex;*/
 // }
  public double decrementIndex() {
    shooterIndex = shooterIndex - 1;
    return shooterIndex;
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

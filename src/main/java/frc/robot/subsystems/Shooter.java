// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private static Shooter _instance = new Shooter();
  double limeLightDistance, shooterIndex;
  boolean fineAdjustment = false;

  public void toggle() {
    fineAdjustment = !fineAdjustment;
  }

  public static Shooter getInstance() {
    return _instance;
  }

  public Shooter() {
    shooterIndex = 6;
  }

  public double index() {
    return shooterIndex;
  }

  public void incrementIndex() {
    if (fineAdjustment) {
      shooterIndex += Constants.kFineAdjustment;
    } else {
      shooterIndex += Constants.kCoarseAdjustment;
    }
  }

  public void decrementIndex() {
    if (fineAdjustment) {
      shooterIndex -= Constants.kFineAdjustment;
    } else {
      shooterIndex -= Constants.kCoarseAdjustment;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

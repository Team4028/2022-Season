// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private static Vision _instance = new Vision();
  /** Creates a new Vision. */
  public Vision() {}

  public void toggleCam() {
    System.out.println(SmartDashboard.getString("CamSelection", " "));
    System.out.println(VisionConstants.kCamera1Url);
    if (SmartDashboard.getString("CamSelection", " ").compareTo(VisionConstants.kCamera1Url) == 0) {
      System.out.println("setting Camera 2");
      SmartDashboard.putString("CamSelection", VisionConstants.kCamera2Url);
    } else {
      System.out.println("setting Camera 1");
      SmartDashboard.putString("CamSelection", VisionConstants.kCamera1Url);
    }
  }

  public static Vision getInstance(){
    return _instance;
  }
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private static Vision _instance = new Vision();
  HttpCamera infeedCam, shooterCam;
  boolean useInfeedCam = true;

  VideoSink dashCam;

  /** Creates a new Vision. */
  public Vision() {
    infeedCam = new HttpCamera("infeedCamera", VisionConstants.kCamera1Url, HttpCameraKind.kMJPGStreamer);
    shooterCam = new HttpCamera("shooterCamera", VisionConstants.kCamera2Url, HttpCameraKind.kMJPGStreamer);

    dashCam = CameraServer.addSwitchedCamera("Dashboard Cam");
    setInfeedCamera();

  }

  public void toggleCam() {
    useInfeedCam = !useInfeedCam;
    if (useInfeedCam) {
        dashCam.setSource(infeedCam);
    } else {
        dashCam.setSource(shooterCam);
    }
  }

  public void setShooterCamera() {
      useInfeedCam = false;
      dashCam.setSource(shooterCam);
  }

  public void setInfeedCamera() {
    useInfeedCam = true;
    dashCam.setSource(infeedCam);
  }

  public static Vision getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

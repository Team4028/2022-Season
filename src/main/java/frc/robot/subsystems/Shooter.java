// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.LinkedHashMap;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.SubsystemConstants;
// import frc.robot.Constants.VBusConstants;

// public class Shooter extends SubsystemBase {
//   private TalonFX _frontMotor;
//   private TalonFX _backMotor;
//   /** Creates a new Shooter. */
//   private static Shooter _instance = new Shooter();
//   private Limelight _l;
//   double limelightDistance, shooterIndex = 6;
//   boolean fineAdjustment = false;
//   boolean accept = true;

//   public double getLimelightDistance() {
//     limelightDistance = _l.distance() / 12;
//     return limelightDistance;
//   }

//   public void toggle() {
//     fineAdjustment = !fineAdjustment;
//   }

//   public boolean getFineAdjustment() {
//     return fineAdjustment;
//   }

//   public static Shooter getInstance() {
//     return _instance;
//   }

//   public void acceptLimelight() {
//     if (accept) {
//       shooterIndex = limelightDistance; // TODO: round
//     } else {
//       shooterIndex = 6; // TODO: default in constants
//     }
//     accept = !accept;
//     SmartDashboard.putString("Accept Limelight Mode", (accept ? "Accept Limelight" : "Reset to Default"));
//     // TODO: better naming
//   }

//   public Shooter() {
//     _frontMotor = new TalonFX(SubsystemConstants.SHOOTER_FRONT_MOTOR_ID);
//     _backMotor = new TalonFX(SubsystemConstants.SHOOTER_BACK_MOTOR_ID);
  
//     _backMotor.setInverted(InvertType.InvertMotorOutput);
    
//     _l = Limelight.getInstance();
//   }

//   public void runShooterMotors(){
//     _frontMotor.set(ControlMode.PercentOutput, VBusConstants.kShooterFront);
//     _backMotor.set(ControlMode.PercentOutput, VBusConstants.kShooterBack);//.67);

//     SmartDashboard.putNumber("Front Motor RPM", _frontMotor.getSelectedSensorVelocity() * 600 / 4096);
//     SmartDashboard.putNumber("Back Motor RPM", _backMotor.getSelectedSensorVelocity() * 600 / 4096);
//   }

//   public void stopShooterMotors(){
//     _frontMotor.set(ControlMode.PercentOutput, 0);
//     _backMotor.set(ControlMode.PercentOutput, 0);
//   }

//   public void shiftShooterVbus(double frontshift, double backshift){
//     // shooterBackVbus += backshift;
//     // shooterFrontVbus += frontshift;
//   }

//   public double index() {
//     return shooterIndex;
//   }

//   public void incrementIndex() {
//     if (fineAdjustment) {
//       shooterIndex += Constants.kFineAdjustment;
//     } else {
//       shooterIndex += Constants.kCoarseAdjustment;
//     }
//   }

//   public void decrementIndex() {
//     if (fineAdjustment) {
//       shooterIndex -= Constants.kFineAdjustment;
//     } else {
//       shooterIndex -= Constants.kCoarseAdjustment;
//     }
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }

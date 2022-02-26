// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.EncoderConstants;
// import frc.robot.Constants.VBusConstants;
// import frc.robot.subsystems.Conveyor;


// public class RunConveyorWithEncoder extends CommandBase {
//   private Conveyor _Conve = Conveyor.get_instance();
//   /** Creates a new RunWithEncoder. */
//   public RunConveyorWithEncoder() {
//     addRequirements(_Conve);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     _Conve.resetEncoder();
//     _Conve.setIsTargetReached();

//     System.out.println("======== Initializing ========");
//     System.out.println("Target Reached: " + _Conve.getIsTargetReached());
//     System.out.println("INITIALIZE Encoder value " + _Conve.getEncoderPosition());

//     _Conve.runConveyorMotorWithEncoder(EncoderConstants.kConveyOne, VBusConstants.kConveyOne);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     _Conve.runConveyorMotorWithEncoder(EncoderConstants.kConveyOne, VBusConstants.kConveyOne);
//     // System.out.println("Bruh:" + _Conve.getIsTargetReached());
//     // System.out.println("======== Execute ========");
//     // System.out.println("EXECUTE Encoder value " + _Conve.getEncoderPosition());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("======== End ========");
//     System.out.println("Target Reached: " + _Conve.getIsTargetReached());
//     System.out.println("Encoder Position: " + _Conve.getEncoderPosition());
//     // _Conve.resetEncoder();
//     // _Conve.setIsTargetReached();
//     // _Conve.setEndStuff(); 
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return _Conve.getIsTargetReached();
//   }
// }

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants.VBusConstants;
// import frc.robot.subsystems.Climber;

// public class TraversalBarClimb extends SequentialCommandGroup {
//   Climber climber = Climber.getInstance();
//   /** Add your docs here. */
//   public TraversalBarClimb() {
//     addCommands(
//     // new HighBarClimb(), // high bar first!
//     new WaitCommand(.25), 
//     new ToggleGrippy(),  //ready to latch
//     new WaitCommand(2), //NOTE: we end high bar climb READY to toggle grippy!
//     new MoveArm(VBusConstants.kClimberFast, 140), // pull grippy up until it clears traverse bar
//     new WaitCommand(1),  
//     new ToggleGrippy(), // latch
//     new WaitCommand(.5), 
//     new MoveArm(-VBusConstants.kClimberFast, -5), // pull down to trav
//     new WaitCommand(.25), 
//     new MoveArm(VBusConstants.kClimberSlow, 30), // slowly up until tippy clears
//     new WaitCommand(.25),  
//     new MoveArm(-VBusConstants.kClimberFast, -30)); // pull down until tippy clears & releases
//     //instant command for solenoids
//     //,command,new command, new command, new command
//   }


// }

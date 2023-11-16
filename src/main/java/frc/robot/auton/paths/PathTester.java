// package frc.robot.auton.paths;

// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.auton.util.AutonBase;
// import frc.robot.auton.util.AutonUtil;
// import frc.robot.subsystems.drive.DriveSubsystem;

// public class PathTester extends AutonBase {
//     public PathTester(DriveSubsystem drive) {
    
//     PathPlannerTrajectory parkingPath = AutonUtil.loadTrajectory("Test", 2.0, 5.0);

//     Pose2d initialPose = AutonUtil.initialPose(parkingPath);

//     if (parkingPath == null) {
//         System.out.println("Path not found");
//         return;
//     }

//     addCommandsWithLog("Top park",
//      new InstantCommand(() -> drive.resetOdometry(initialPose), drive).withName("Reset Odometry"), drive.trajectoryFollowerCommand(parkingPath).andThen(new WaitCommand(0.2)));
    
// }
//     @Override
//   public void initSendable(SendableBuilder builder) {
//     super.initSendable(builder);
//   }
// }

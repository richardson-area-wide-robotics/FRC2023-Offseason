// package frc.robot.auton.util;

// import java.util.HashMap;
// import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.FollowPathWithEvents;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutonUtil {
//     private static HashMap<String, PathPlannerTrajectory> m_cache = new HashMap<>();
//     private static HashMap<String, Command> eventMap = new HashMap<>();
//     private static List<PathPlannerTrajectory> pathGroup;

//     private static String cacheKey(String name, double maxVel, double maxAccel, boolean reversed) {
//       return new StringBuilder()
//           .append(name)
//           .append(maxVel)
//           .append(maxAccel)
//           .append(reversed)
//           .toString();
//     }

//     public static Pose2d initialPose(PathPlannerTrajectory trajectory) {
//         return new Pose2d(
//             trajectory.getInitialState().poseMeters.getTranslation(),
//             trajectory.getInitialState().holonomicRotation);
//       }
    
//       public static PathPlannerTrajectory loadTrajectory(String name, double maxVel, double maxAccel) {
//         return loadTrajectory(name, maxVel, maxAccel, false);
//       }
    
//       public static PathPlannerTrajectory loadTrajectory(
//           String name, double maxVel, double maxAccel, boolean reversed) {
//         String cacheKey_ = cacheKey(name, maxVel, maxAccel, reversed);
//         if (m_cache.containsKey(cacheKey_)) {
//           return m_cache.get(cacheKey_);
//         }
    
//         PathPlannerTrajectory trajectory = PathPlanner.loadPath(name, maxVel, maxAccel, reversed);
        
    
//         if (trajectory == null) {
//             DriverStation.reportError("Auton Path... Failed to load trajectory paths: {}" + name, false);
//         }

//         System.out.println("Loaded auton {}" + name + " max velocity: {}" + maxVel + "max accel: {}" + maxAccel + "reversed: {}" + reversed + "starting state {}" + trajectory.getInitialState());
    
//         /*
//          * Store trajectories. This is done since multiple autons may load the same trajectories,
//          * no need to load it again and again. If anything funky with trajectories is happening,
//          * comment out the below. (e.g. I don't see any stored state in the trajectory, but it
//          * is not clear if that is true, and if calling the trajectory stores any state.
//          * Investigate later.)
//          */
//         m_cache.put(cacheKey_, trajectory);
    
//         return trajectory;
//       }

//       /**
//        * Loads a group of paths that are in the trajectory folder
//        * @param namethe name of the json file 
//        * @param constraints The PathConstraints (max velocity, max acceleration) of the first path in the group
//        * @param extraConstraints The PathConstraints (max velocity, max acceleration) of the remaining paths in the group. If there are less constraints than paths, the last constrain given will be used for the remaining paths.
//        * @return A List of all generated paths in the group
//        */
//       public static List<PathPlannerTrajectory> loadTrajectoryGroup(String name, PathConstraints constraints, PathConstraints... extraConstraints){
//         pathGroup = PathPlanner.loadPathGroup(name, constraints, extraConstraints);
//         return pathGroup;
//       }

//       /**
//        * @return gets the previous loaded pathgroup
//        */
//       public static List<PathPlannerTrajectory> getPathGroup(){
//         return pathGroup;
//       }
      

//       /**
//        * Will run the FollowEventCommand that will help reduce redundant code in the auton classes.
//        * @param trajectoryCommand the command that will follow the trajectory
//        * @param pathNum the number of the path that is being followed
//        */
//       public static FollowPathWithEvents followEventCommand(Command trajectoryCommand, PathPlannerTrajectory pathNum){
//         return new FollowPathWithEvents(trajectoryCommand, pathNum.getMarkers(), getEventMap());
//       }

//       /**
//        * Adds events to the event map that can be used gloablly in any class.
//        * The method will add the event to the event map if it does not already exist.
//        * @param name the name of the event
//        * @param event the event to be added to the map 
//        */
//       public static void addEvent(String name, Command event){
//         if(!eventMap.containsKey(name)){
//           eventMap.put(name, event);
//         }
//       }

//       /**
//        * @return the event map
//       */
//       public static HashMap<String, Command> getEventMap(){
//         return eventMap;
//       }


// }
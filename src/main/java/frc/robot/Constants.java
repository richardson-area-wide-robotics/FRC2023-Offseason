// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.FFGains;
import frc.lib.controller.PIDGains;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kControllerDeadband = 0.1;
    
  }
  public static final class SwerveDriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    /*
     * Chassis configuration
     */
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // The kinematics for the robot drivetrain
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kDriveRadius = Math.sqrt(Math.pow(kWheelBase, 2) + Math.pow(kTrackWidth, 2)); // TODO: DO NOT RUN WITHOUT CHECKING THIS

    /*
     * Spark Max and encoder constents for MAXSwerve modules
     */
    public static final class FrontLeftModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 2;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 1;
      // The Angular offset in radians for the steer encoder in radians
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      // creating the swerve module constants for the front left module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kFrontLeftChassisAngularOffset);
    }

    public static final class FrontRightModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 8;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 7;
      // The Angular offsets in radians for the steer encoder
      public static final double kFrontRightChassisAngularOffset = 0;
      // creating the swerve module constants for the front right module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kFrontRightChassisAngularOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeftModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 4;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 3;
      // The Angular offsets in radians for the steer encoder
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      // creating the swerve module constants for the back left module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kBackLeftChassisAngularOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class BackRightModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 6;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 5;
      // The Angular offsets in radians for the steer encoder
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;
      // creating the swerve module constants for the back right module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS =
          new SwerveModuleConstants(
              kDriveMotorCANID, kSteerMotorCANID, kBackRightChassisAngularOffset);
    }

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    public static final boolean kDrivingIsInverted = false;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kFreeSpeedRpm = 5676.0;
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15.0);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0.0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final PIDGains kDrivingPIDGains =
        new PIDGains(0.04, 0, 0); // TODO: tune values for the driving motor
    public static final FFGains kDrivingFFGains = new FFGains(0,0, 0);
    public static final double kDrivingFF = 1.0 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1.0;
    public static final double kDrivingMaxOutput = 1.0;

    public static final PIDGains kTurningPIDGains =
        new PIDGains(3.75, 0, 0.35); // TODO: tune values for the turning motor
    public static final FFGains kTurningFFGains = new FFGains(0,0, 0);
    public static final double kTurningFF = 0.0; // TODO: tune values for Feed Forward
    public static final double kTurningMinOutput = -1.0;
    public static final double kTurningMaxOutput = 1.0;

    
    public static final PIDGains kVisionTurningPIDGains =
        new PIDGains(1.0, 0, 0.01); // TODO: tune values for Vision auto-turning

    public static final double MAX_LOCKED_ON_SPEED = 0.33;
    
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 10; // amps
  }
  public static final class AutoConstants {
     public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;    
    
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PIDController kPXController = new PIDController(1.0, 0, 0.35);
    public static final PIDController kPYController = new PIDController(0.001, 0, 0.00075);
    public static final PIDController kPThetaController = new PIDController(8.0, 0, 0.75);

    // PID constants for new Pathplanner code
    public static final PIDGains kTranslationGains = new PIDGains(0.0, 0.0, 0.0);
    public static final PIDGains kRotationGains = new PIDGains(0.0, 0.0, 0.0);

    public static final PIDGains kMovingPIDGains =
        new PIDGains(0.3,0, 0.01);
    public static final double offset = 0.00315;
  }
  public static final class Intake {
    public static final boolean kIntakeInverted = true;
    public static final int kIntakeID = 12;
    public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;
    public static final int kIntakeCurrentLimit = 50; // amps
	  public static final double kIntakeSpeed = 1.0;
    public static final double kOutakeSpeed = -1.0;
    public static final double kConeIdleSpeed = 0.05;
    public static final double kCubeIdleSpeed = -0.05;
  }
  public static final class ArmConstants {
    // // Arm limits //TODO: tune these values/or remove them
    // public final static float ARM_REVERSE_LIMIT = 34.0f;
    // public final static float ARM_FORWARD_LIMIT = 0.0f;
    
    /* Left motor constants - Follower */
    // CAN ID
    public final static int LEFT_MOTOR_CAN_ID = 10;
    // Motor Direction
    public static final boolean LEFT_ARMMOTOR_INVERTED = true; 
    
    /* Right motor constants - Leader (Abs Encoder) - Contains all main value for arm motors */
    // CAN ID
    public final static int RIGHT_MOTOR_CAN_ID = 9; 
    // Motor Direction
    public static final boolean RIGHT_ARMMOTOR_INVERTED = false;
    // Idle mode 
    public static final IdleMode kRightMotorIdleMode = IdleMode.kBrake;
    // Current Limit
    public static final int kArmMotorCurrentLimit = 40;
    public static final int kMovingArmMotorCurrentLimit = 60;
    // Absolute encoder - relative encoder conversions 
    public static final double kArmEncoderPositionFactor = 2 * Math.PI;
    public static final double kArmEncoderVelocityFactor = (2 * Math.PI) / 60.0;
    // Arm PID values
    public static final PIDGains ARM_PID_GAINS = new PIDGains(2.0, 0.0, 0.0); // retunne these values
    
    /* Elbow motor constants */
    // CAN ID
    public final static int ELBOW_MOTOR_CAN_ID = 11;
    // Motor Direction
    public static final boolean ELBOW_MOTOR_INVERTED = true;
    // Idle mode
    public static final IdleMode kElbowMotorIdleMode = IdleMode.kBrake;
    // Current Limit
    public static final int kElbowMotorCurrentLimit = 60;
    // Absolute encoder - relative encoder conversions
    public static final double kElbowEncoderPositionFactor = 2 * Math.PI;
    public static final double kElbowEncoderVelocityFactor = (2 * Math.PI) / 60.0;
    // Elbow PID values
    public static final PIDGains ELBOW_PID_GAINS = new PIDGains(3.5, 0.0, 0.65); // retunne these values
    
    
    // PID constants using custom PID gains class //TODO: tune PID values
    public static final ArmFeedforward ARM_MOTOR_FEEDFORWARD = new ArmFeedforward(0.0,0.72,1.56, 0.08);
    public static final double ARM_FF = 0.0;
    
    public static final ArmFeedforward ELBOW_MOTOR_FEEDFORWARD = new ArmFeedforward(0.0,0.77,0.7,0.04);
    public static final double ELBOW_FF = 0.0;
    public static final double MIN_OUTPUT = -1.0;
    public static final double MAX_OUTPUT = 1.0;
    
    // Encoder offsets - radians 
    public static final double ARM_ENCODER_OFFSET = 0;
    public static final double ELBOW_ENCODER_OFFSET = 0;
    
    public static final double ARM_STOWED = .622; // new value
    public static final double ELBOW_STOWED = .274; // new value
    
    public static final double ARM_PICK_UP_TCONE = .723; // new value
    public static final double ELBOW_PICK_UP_TCONE = .729; // new value
    
    public static final double ARM_PICK_UP_CONE = 0.618; 
    public static final double ELBOW_PICK_UP_CONE = 0.281;
    
    public static final double ARM_PICK_UP_CUBE = 0.679; // new value
    public static final double ELBOW_PICK_UP_CUBE = 0.59; // new value
    
    public static final double ARM_SCORE_CUBE_LOW = ARM_STOWED; // new value
    public static final double ELBOW_SCORE_CUBE_LOW = 0.5; // new value
    
    public static final double ARM_SCORE_CONE_LOW = ARM_STOWED; // new value
    public static final double ELBOW_SCORE_CONE_LOW = ELBOW_SCORE_CUBE_LOW; // new value
    
    public static final double ARM_SCORE_CONE_MID = 0.317; // new value
    public static final double ELBOW_SCORE_CONE_MID = 0.568; // new value
    
    public static final double ARM_SCORE_CUBE_MID = 0.697; // new value
    public static final double ELBOW_SCORE_CUBE_MID = 0.487; // new value
    
    public static final double ARM_SCORE_CONE_HIGH = 0.288; // new value
    public static final double ELBOW_SCORE_CONE_HIGH = 0.629;// new value
    
    public static final double ARM_SCORE_CUBE_HIGH = 0.794; // new value
    public static final double ELBOW_SCORE_CUBE_HIGH = 0.534; // new value
    
    public static final double ARM_PICK_UP_SHELF = 0.636; // new value
    public static final double ELBOW_PICK_UP_SHELF = 0.415; // new value
    
    public static final double ARM_PICK_UP_DSHELF = 0.795;
    public static final double ELBOW_PICK_UP_DSHELF = 0.219; 
    
    // public static final double ELBOW_IDLE = .83;
    public static final double ARM_TOLERANCE = 0; // TODO: tune this value
    public static final double ELBOW_TOLERANCE = 0;
      }
  
public static final boolean kCompetitionMode = false;

  }


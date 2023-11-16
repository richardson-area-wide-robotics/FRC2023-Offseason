// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.JoystickUtil;
import frc.robot.Constants.IOConstants;
// import frc.robot.auton.paths.PathTester;
import frc.robot.auton.util.AutoChooser;
import frc.robot.commands.armCommands.PositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.manipulator.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final AHRS m_gyro = new AHRS();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final PositionCommand armPositions = new PositionCommand(arm);
  private Boolean intakeMode = false;
  private double speed = -0.1;
  private double intakeSpeed = 1.0;
  private double outtakingSpeed = -1.0;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(IOConstants.kDriverControllerPort);
  XboxController m_driverControllerSP = new XboxController(IOConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(IOConstants.kOperatorControllerPort);

  {
    // AutoChooser.setDefaultAuton(new PathTester(m_robotDrive));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   
    // Configure the trigger bindings
    configureDriverBindings(); 
    configureOperatorBindings();   
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriverBindings() {

    // TODO: remove this before merging
    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(balance);

    // set up for driving controls
    DoubleSupplier moveForward =  () -> MathUtil.applyDeadband(
      -m_driverController.getLeftY(), Constants.IOConstants.kControllerDeadband);
     DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
      -m_driverController.getLeftX(), Constants.IOConstants.kControllerDeadband);
    
    // Configure default commands
    /* 
       * ---Driving Controls for the driver 
       * The left stick on Xbox controller controls the translation of the robot - 1
       * The right stick controls the rotation of the robot - 12
       */
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () ->
          m_robotDrive.drive(
          moveForward.getAsDouble(),
          moveSideways.getAsDouble(),
          JoystickUtil.squareAxis(
          -m_driverController.getRightX()),
          true), m_robotDrive));

      /*
       * ---Reset button and X mode button
       * left stick button on controller controls the re-zeroing of the heading 
       * right stick button on controller controls the X mode of the robot
       */

       m_driverController.rightStick().onTrue(new InstantCommand(()-> m_robotDrive.zeroHeading()));
       m_driverController.leftStick().onTrue(new InstantCommand(()-> m_robotDrive.setX()));

        /*
     * ---Intaking Controls
     * Left trigger - intaking - 16
     * Right trigger - outaking - 13
     * Up on dpad on the back of Xbox controller controls the toggle between cone and intake state using Intake - P1
     */

    intake.setDefaultCommand(new RunCommand(()->intake.setIntakeSpeed(speed), intake));

    m_driverController.leftTrigger().whileTrue(new RunCommand(()->intake.setIntakeSpeed(intakeSpeed), intake));
    m_driverController.rightTrigger().whileTrue(new RunCommand(()->intake.setIntakeSpeed(outtakingSpeed), intake));

      // Stow
      m_driverController.b().onTrue(armPositions.armStowCommand());
      // Tipped pick up
      m_driverController.y().onTrue(armPositions.armPickUpTconeCommand()).onTrue(new InstantCommand(()->{ this.intakeMode = false;}));
      // // Standing Cone 
      m_driverController.a().onTrue(armPositions.armPickUpStandingConeCommand());
      // // Pick up Cube 
      m_driverController.x().onTrue(armPositions.armPickUpCubeCommand()).onTrue(new InstantCommand(()->{ this.intakeMode = true;}));

      // Shelf
      m_driverController.leftBumper().onTrue(armPositions.armShelfCommand());
      // Double Shelf
      m_driverController.rightBumper().onTrue(armPositions.armDShelfCommand());

      // Score Cone High
      m_driverController.pov(0).onTrue(armPositions.armScoreConeHighCommand());
      // Score Cone Mid
      m_driverController.pov(90).onTrue(armPositions.armScoreConeMidCommand());
      // Score Cube High
      m_driverController.pov(270).onTrue(armPositions.armScoreCubeHighCommand());
      // Score Cube Mid 
      m_driverController.pov(180).onTrue(armPositions.armScoreCubeMidCommand());
     
  }

  private void configureOperatorBindings(){
   /*
    * All of the operator controls will go here 
    */
    m_operatorController.y().onTrue(armPositions.armScoreConeHighCommand());
    m_operatorController.b().onTrue(armPositions.armScoreConeMidCommand());
    m_operatorController.x().onTrue(armPositions.armScoreCubeHighCommand());
    m_operatorController.a().onTrue(armPositions.armScoreCubeMidCommand());
  }

   /** Run a function at the start of auton. */
   public void autonInit(){
    // m_robotDrive.calibrateGyro();
    // m_robotDrive.stop();
    this.globalEventList();
  }

  /** Creates the Global event list for the autonomous paths */
  public void globalEventList(){}
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    System.out.println("Auton Selected" + AutoChooser.getAuton().getName());
    return AutoChooser.getAuton();
  }

  /**
   * Use this method to pass anythign to the dashboard 
   * 
   * Reduces multi method use to Shuffleboard
   */
  public void putDashboard(){
    arm.armShuffleBoard();

    if (!intakeMode){
      speed = 0.1;
    }
    else {
      speed = -0.1;
    }

    if(!intakeMode){
      intakeSpeed = 1.0;
      outtakingSpeed = -0.25;
    } else {
     intakeSpeed = -1.0;
     outtakingSpeed = 1.0;
    }
    // m_robotDrive.putNumber();
    // SmartDashboard.putNumber("filtered PoseX", m_robotDrive.getPose().getX());
    // SmartDashboard.putNumber("filtered PoseY", m_robotDrive.getPose().getY());
  }

    /**
   * Sets the idle mode for the arm and intake joints
   * 
   * Used to make the robot arm easier to move when disabled
   */
  public void setIdleMode(IdleMode idleMode){
    arm.getSparkStatus(idleMode);
}

/** Run a function during autonomous to get run time of autonomous. */
public void autonPeriodic(){
  SmartDashboard.putNumber("Auton Time", Timer.getFPGATimestamp());

}
}
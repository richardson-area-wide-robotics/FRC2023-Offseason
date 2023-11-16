package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    public boolean isArmStowed = true;
    public boolean isArmExtended = false;
    public boolean overrideMode = false;

     // Shoulder Motors
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private boolean normalStow;

  // Elbow Motors
  private CANSparkMax elbowMotor;

  // PID Controllers
  private PIDController armPID;
  private PIDController elbowPID;  
  private SparkMaxPIDController armPIDController;
  private SparkMaxPIDController elbowPIDController;  

  // Encoders
  private AbsoluteEncoder armEncoder;
  private AbsoluteEncoder elbowEncoder;

  // Arm Positions
  private double currentArmPosition;
  private double currentElbowPosition;

  // last arm position
  private double lastArmPosition;
  private double lastElbowPosition;

  public double armPosition;

  private SimpleWidget armAbsPos;
  private SimpleWidget armRelativePos;
  private SimpleWidget elbowAbsPos;
  private SimpleWidget elbowRelativePos;

    // set up the arm congfiguration
  public void armConfig(CANSparkMax motor, AbsoluteEncoder enc){
    // restore factory defaults
    motor.restoreFactoryDefaults();
    // set motor basic values
    motor.setIdleMode(Constants.ArmConstants.kRightMotorIdleMode);
    // set the arm PID controllers
    motor.setSmartCurrentLimit(Constants.ArmConstants.kArmMotorCurrentLimit);
    armPIDController = motor.getPIDController();
    armPIDController.setFeedbackDevice(enc);
    armPIDController.setPositionPIDWrappingEnabled(false);
    armPID = new PIDController(Constants.ArmConstants.ARM_PID_GAINS.P, Constants.ArmConstants.ARM_PID_GAINS.I, Constants.ArmConstants.ARM_PID_GAINS.D);
    armPIDController.setP(armPID.getP());
    armPIDController.setI(armPID.getI());
    armPIDController.setD(armPID.getD());
    armPIDController.setOutputRange(
        Constants.ArmConstants.MIN_OUTPUT, Constants.ArmConstants.MAX_OUTPUT);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);
  }

  // set up the elbow congfiguration
  public void elbowConfig(CANSparkMax motor, AbsoluteEncoder enc){
  // restore factory defaults
  motor.restoreFactoryDefaults();
  // set motor basic values
  motor.setIdleMode(Constants.ArmConstants.kElbowMotorIdleMode);
  motor.setSmartCurrentLimit(Constants.ArmConstants.kElbowMotorCurrentLimit);
  elbowPIDController = motor.getPIDController();
  elbowPIDController.setFeedbackDevice(enc);
  elbowPIDController.setPositionPIDWrappingEnabled(false);
  elbowPID = new PIDController(Constants.ArmConstants.ELBOW_PID_GAINS.P, Constants.ArmConstants.ELBOW_PID_GAINS.I, Constants.ArmConstants.ELBOW_PID_GAINS.D);
  elbowPIDController.setPositionPIDWrappingMinInput(
        0.0);
    elbowPIDController.setPositionPIDWrappingMaxInput(
        2 * Math.PI);

  elbowPIDController.setP(elbowPID.getP());
  elbowPIDController.setI(elbowPID.getI());
  elbowPIDController.setD(elbowPID.getD());
  elbowPIDController.setOutputRange(
      Constants.ArmConstants.MIN_OUTPUT, Constants.ArmConstants.MAX_OUTPUT);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);
  }

    public Arm(){
    // motor type for right motor
    leftMotor = new CANSparkMax(Constants.ArmConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.ArmConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    elbowMotor = new CANSparkMax(Constants.ArmConstants.ELBOW_MOTOR_CAN_ID, MotorType.kBrushless);

    // setting the Absolute Encoder for the SparkMax
    armEncoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
    elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second

    elbowEncoder.setPositionConversionFactor(Constants.ArmConstants.kElbowEncoderPositionFactor);
    elbowEncoder.setVelocityConversionFactor(Constants.ArmConstants.kElbowEncoderVelocityFactor);

    // setting the motor configuration
    armConfig(rightMotor, armEncoder);
    elbowConfig(elbowMotor, elbowEncoder);

    /** 
     * converting the relative encoder conversion factor from rotations to radians 
     * Also need to make the conversion as the motr is not on the same axis as the arm
     * it goes from a 16t sprocket to a 64t sprocket; so need to account for that in the conversion
     * **/
    // rightMotor.getEncoder().setPositionConversionFactor(Constants.ArmConstants.kArmEncoderPositionFactor);
    rightMotor.getEncoder().setVelocityConversionFactor(Constants.ArmConstants.kArmEncoderVelocityFactor * (16.0/64.0));

    armEncoder.setPositionConversionFactor((4096/8192) * 2 * Math.PI);

    rightMotor.setInverted(Constants.ArmConstants.RIGHT_ARMMOTOR_INVERTED);
    elbowMotor.setInverted(true);

    // setting the idle mode setting for the SparkMax, here it is set to brake when idle
    leftMotor.follow(this.rightMotor, true);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
    elbowMotor.burnFlash();

    rightMotor.getEncoder().setPosition(armEncoder.getPosition());
    elbowMotor.getEncoder().setPosition(elbowEncoder.getPosition());

    this.currentArmPosition = Constants.ArmConstants.ARM_STOWED;
    this.currentElbowPosition = Constants.ArmConstants.ELBOW_STOWED;
    
    // set up variables for getting the last know arm and elbow position
    this.lastArmPosition = this.currentArmPosition;
    this.lastElbowPosition = this.currentElbowPosition;

    this.armPosition = 0; // 0 is position stowed, 1 is special stow

    this.normalStow = true;

     armAbsPos = tab.add("Arm Absolute Position", getArmAbsPosition()).withWidget(BuiltInWidgets.kGyro);
     armRelativePos = tab.add("Arm Relative Position", getArmRelPosition()).withWidget(BuiltInWidgets.kGyro);

     elbowAbsPos = tab.add("Elbow Absolute Position", getElbowAbsPosition()).withWidget(BuiltInWidgets.kGyro);
     elbowRelativePos = tab.add("Elbow Relative Position", getElbowRelPosition()).withWidget(BuiltInWidgets.kGyro);


    }

    /* Get Absolute position of arm encoder */
    public double getArmAbsPosition() {
        return armEncoder.getPosition();
    }

    /* Get relative spark encoder position of arm */
    public double getArmRelPosition() {
        return rightMotor.getEncoder().getPosition();
    }

    /* Get Absolute position of elbow encoder */
    public double getElbowAbsPosition() {
        return elbowEncoder.getPosition();
    }

    /* Get relative spark encoder position of elbow */
    public double getElbowRelPosition() {
        return elbowMotor.getEncoder().getPosition();
    }

    /* Arm is at target */
    public boolean armAtTarget() {
        return Math.abs(getArmAbsPosition() - currentArmPosition) < Constants.ArmConstants.ARM_TOLERANCE;
    }

    /* Elbow is at target */
    public boolean elbowAtTarget() {
        return Math.abs(getElbowAbsPosition() - currentElbowPosition) < Constants.ArmConstants.ELBOW_TOLERANCE;
    }

    /* Add whatever in method to the shuffleboard */
    public void armShuffleBoard(){
        // this.armAbsPos.getEntry();
        // this.elbowAbsPos.getEntry();
        // this.elbowRelativePos.getEntry();
        // this.armRelativePos.getEntry();
        SmartDashboard.putNumber("Arm Absolute Position", getArmAbsPosition());
        SmartDashboard.putNumber("Arm Relative Position", getArmRelPosition());
        SmartDashboard.putNumber("Elbow Absolute Position", getElbowAbsPosition());
        SmartDashboard.putNumber("Elbow Relative Position", getElbowRelPosition());

    }

    public void setArmPosition(double position) {
        lastArmPosition = currentArmPosition;
        currentArmPosition = position;
      }

    
    public void getSparkStatus(IdleMode mode){
    leftMotor.setIdleMode(mode);
    rightMotor.setIdleMode(mode);
    elbowMotor.setIdleMode(mode);
    }
    
      public void setElbowPosition(double position) {
        lastElbowPosition = currentElbowPosition;
        currentElbowPosition = position;
        
      }

    public void resetEncoders(){
        rightMotor.getEncoder().setPosition(0);
        elbowMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic(){
        // set up a watchdog to make sure relative position on spark encoders matches absolute position;
        // if (Math.abs(getArmAbsPosition() - getArmRelPosition()) > 0.1 /* may need to tune this number */) {
        //     rightMotor.getEncoder().setPosition(getArmAbsPosition());
        // }

        // if (Math.abs(getElbowAbsPosition() - getElbowRelPosition()) > 0.1 /* may need to tune this number */) {
        //     elbowMotor.getEncoder().setPosition(getElbowAbsPosition());
        // }
        armPIDController.setReference(currentArmPosition, ControlType.kPosition);
        elbowPIDController.setReference(currentElbowPosition, ControlType.kPosition/* , 1, elbowFF.calculate(elbowPID.getSetpoint().position, elbowPID.getSetpoint().velocity)*/);

        // armShuffleBoard();
    }
}
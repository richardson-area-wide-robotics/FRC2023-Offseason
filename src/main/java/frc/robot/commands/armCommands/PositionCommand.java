package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class PositionCommand extends SequentialCommandGroup {
    ElbowPosition elbowCommand;
    ShoulderPosition shoulderPosition;
    Arm arm;
    // Intake intake;

    public PositionCommand(Arm armMech){
        this.arm = armMech;
    }

    public Command armStowCommand(){
        return new SequentialCommandGroup(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_STOWED).andThen(new WaitCommand(0.1)).andThen(new ShoulderPosition(arm, Constants.ArmConstants.ARM_STOWED)));
    }

    public Command armShelfCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_PICK_UP_SHELF).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_PICK_UP_SHELF)));
    }

    public Command armDShelfCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_PICK_UP_DSHELF).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_PICK_UP_DSHELF)));
    }

    /*
     * Arm cube positions
     * 1. Score cube low
     * 2. Score cube mid
     * 3. Score cube high
     * 4. Pick Up cube
     */
    public Command armScoreCubeLowCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_SCORE_CUBE_LOW).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_SCORE_CUBE_LOW)));
    }

    public Command armScoreCubeMidCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_SCORE_CUBE_MID).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_SCORE_CUBE_MID)));
    }

    public Command armScoreCubeHighCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_SCORE_CUBE_HIGH).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_SCORE_CUBE_HIGH)));
    }

    public Command armPickUpCubeCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_PICK_UP_CUBE).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_PICK_UP_CUBE)));
    }

    /*
     * 1. Score cone low
     * 2. Score cone mid
     * 3. Score cone high
     * 4. Pick up Tcone
     * 5. Pick up standing cone
     */
    public Command armScoreConeLowCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_SCORE_CONE_LOW).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_SCORE_CONE_LOW)));
    }

    public Command armScoreConeMidCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_SCORE_CONE_MID).andThen(new WaitCommand(0.05)).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_SCORE_CONE_MID)));
    }

    public Command armScoreConeHighCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_SCORE_CONE_HIGH).andThen(new WaitCommand(0.05)).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_SCORE_CONE_HIGH)));
    }

    public Command armPickUpTconeCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_PICK_UP_TCONE).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_PICK_UP_TCONE)));
    }

    public Command armPickUpStandingConeCommand(){
        return new SequentialCommandGroup(new ShoulderPosition(arm, Constants.ArmConstants.ARM_PICK_UP_CONE).andThen(new WaitCommand(0.1)).andThen(new ElbowPosition(arm, Constants.ArmConstants.ELBOW_PICK_UP_CONE)));
    }

}
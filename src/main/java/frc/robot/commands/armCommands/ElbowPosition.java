package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ElbowPosition extends Command {
    private Arm arm;
    private double position;
    
    public ElbowPosition(Arm arm, double position) {
        this.arm = arm;
        this.position = position;
        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        arm.setElbowPosition(position);
    }

    // TODO: Make this better and more accurate
    @Override
    public boolean isFinished() {
        return arm.getArmAbsPosition() >= position - .1 && arm.getElbowAbsPosition() <= position + .1;
    }
}
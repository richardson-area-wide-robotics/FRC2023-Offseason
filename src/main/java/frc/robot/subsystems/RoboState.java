package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.localization.Localizer;
import frc.robot.subsystems.drive.DriveSubsystem;



public class RoboState {

    Double Angle;
    DriveSubsystem driveSys;
    Localizer localizer;
    DoubleSupplier Ytranslation;
    DoubleSupplier Xtranslation;


   
    public Double getCamAngle(Double Angle) {
        this.Angle = Angle;
        return Angle;
    }

    //Change variable type to return any information needed
    //Class types are currently place holders

    //what values are needed from localizer?
    public void localizer(Localizer localizer) {
        this.localizer = localizer;
    }
    
    public void drive(DoubleSupplier Xtranslation, DoubleSupplier Ytranslation){
        this.Xtranslation = Xtranslation;
        this.Ytranslation = Ytranslation;
    }
    
    public DoubleSupplier getXtranslation() {
        return Xtranslation;
    }

    public DoubleSupplier getYtranslation() {
        return Ytranslation;
    }



}
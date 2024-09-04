package com.rithek.apriltaglocalization.Localizers;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.rithek.apriltaglocalization.AprilTagReader;
import com.rithek.apriltaglocalization.NoAprilTagFoundException;


public abstract class AprilTagCorrectionLocalizer extends DualMethodLocalizer {
    private AprilTagReader aprilTag;

    public AprilTagCorrectionLocalizer(AprilTagReader aprilTagSensor){

        this.aprilTag = aprilTagSensor;
    }

    @Override
    public void correctError(){
        try {
            Twist2dDual<Time> reading = aprilTag.readTag();
            Twist2dDual<Time> curPos = updateMainLocalizer();
            vecError = reading.line.minus(curPos.line);
            headingError = Math.toDegrees(reading.angle.value()) - curPos.angle.value();
        } catch (NoAprilTagFoundException ignored){

        }

    }

    public void setNewAprilTagSensor(AprilTagReader aprilTagReader){
        aprilTag = aprilTagReader;
    }

}

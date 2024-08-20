package com.example.apriltaglocalization;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import static java.lang.Math.*;

import java.util.ArrayList;


public class AprilTagReader {


    private AprilTagProcessor processor;
    private VisionPortal vportal;

    private Telemetry telemetry;



    public void initAprilTagCamera(HardwareMap hardwareMap, String webcamName, Telemetry telemetry) {
        processor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317f,822.317f,319.495f,242.502f)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        vportal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)

                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();


        this.telemetry = telemetry;


    }


    @Deprecated
    public double getIDX(int id){
        ArrayList<AprilTagDetection> detections = processor.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection != null &&detection.id == id) {
                return detection.ftcPose.x;

            }


        }
        return 0;
    }
    @Deprecated
    public double getIDY(int id){
        ArrayList<AprilTagDetection> detections = processor.getFreshDetections();

        for (AprilTagDetection detection : detections) {
            if (detection != null &&detection.id == id) {
                return detection.ftcPose.y;

            }


        }
        return 0;
    }


//This shows the position relative to the APRIL TAG'S COORDINATE AXES, NOT THE CAMERA'S

    /**
     * Takes in a detection and returns the coordinates values WITH RESPECT TO THE APRIL TAG'S COORD SYSTEM
     * @param detection The April Tag Detection
     * @return Returns a Twist2d with the x, y, and heading with respect to the april tag's coordinate system.
     */
    public Twist2d findCoords(@NonNull AprilTagDetection detection) {
        double rawX = detection.ftcPose.x;
        double rawY = detection.ftcPose.y;
        //double height = detection.ftcPose.z;

        double roll =-detection.ftcPose.roll;

        double xReading = rawX;
        double yReading = cos(roll)*rawY;



        //This is in degrees
        //YAW IS THE ROTATION OF THE APRIL TAG!!! NOT THE CAMERA!!!
        double yaw = detection.ftcPose.yaw;
        double botX,botY,heading,xXCoord,yYCoord,xYCoord,yXCoord;
        double xAngle;
        if (xReading > 0) {
            xAngle = yaw + 180;
        } else {
            xAngle = yaw;
        }

        double yAngle;
        double yaw2 = yaw + 90;
        if (yReading > 0) {
            yAngle = yaw2 - 180;
        } else {
            yAngle = yaw2;
        }
        //first letter means the reading and the second letter means the contribution to that output
        xXCoord = cos(toRadians(xAngle)) * xReading;
        xYCoord = sin(toRadians(xAngle)) * xReading;
        yXCoord = cos(toRadians(yAngle)) * yReading;
        yYCoord = sin(toRadians(yAngle)) * yReading;

        heading = 90-yaw;

        botX = xXCoord + yXCoord;
        botY = xYCoord + yYCoord;

        AprilTag tag = AprilTag.getAprilTag(detection.id);
        Twist2d toRet = new Twist2d(new Vector2d(botX+tag.xPos,botY+tag.yPos).times(1),(heading+tag.heading)%360);
        //Twist2dDual<Time> t2DRet= new Twist2dDual<Time>(new Vector2dDual<Time>());
        boolean telcond = telemetry == null;
        if (!telcond) {

            telemetry.addLine("Bot X: "+ botX +"\nBot Y: "+botY+"\nHeading: "+ yaw+ "\nUnit: "+ detection.metadata.distanceUnit);
        }
        return toRet;
    }



    public Twist2dDual<Time> readTag() throws NoAprilTagFoundException{


        ArrayList<AprilTagDetection> detections;
        detections=processor.getDetections();
        Twist2d coords = null;

//            if (detections != null ) {
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                coords = findCoords(detections.get(0));
                telemetry.addLine("======================");
                telemetry.addData("X", coords.line.x);
                telemetry.addData("Y", coords.line.y);
                telemetry.addData("Heading", coords.angle);
                telemetry.addData("ID", detection.id);
                telemetry.addData("Unit", detection.metadata.distanceUnit);
                telemetry.update();
                break;
            }
        }
        //}

        if (coords == null){
            throw new NoAprilTagFoundException();
        }
        Vector2d vector =coords.line;
        double heading =coords.angle;

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[]{vector.y,0}),
                        new DualNum<>(new double[]{vector.x,0})
                ),
                new DualNum<>(new double[]{heading,0}
                )
        );
    }


    public enum AprilTag {
        ONE(1,30,0,0,0),
        TWO(2,30,0,0,0),
        THREE(3,30,0,0,0),
        FOUR(4,30,0,0,0),
        FIVE(5,30,0,0,0),
        SIX(6,30,0,0,0),
        SEVEN(0,0,0,0,0),
        EIGHT(0,0,0,0,0),
        NINE(9,0,0,0,0),
        TEN(10,0,0,0,0);

        final double id, roll, xPos, yPos, heading;

        AprilTag(double id,double roll,double xPos,double yPos,double heading){
            this.id=id;
            this.roll=roll;
            this.xPos=xPos;
            this.yPos=yPos;
            this.heading=heading;
        }

        public static AprilTag getAprilTag(int tag){
            if (tag ==1){
                return  ONE;
            } else if(tag ==2){
                return TWO;
            } else if (tag==3){
                return  THREE;
            } else if (tag ==4){
                return FOUR;
            } else if (tag == 5){
                return FIVE;
            } else if (tag ==6){
                return SIX;
            } else if(tag == 7) {
                return SEVEN;
            } else if (tag == 8){
                return EIGHT;
            } else if (tag ==9 ) {
                return NINE;
            }
            return  TEN;
        }
    }

}

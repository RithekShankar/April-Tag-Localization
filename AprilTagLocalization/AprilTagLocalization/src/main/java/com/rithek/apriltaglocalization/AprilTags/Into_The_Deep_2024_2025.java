package com.rithek.apriltaglocalization.AprilTags;


/**
 * Heading instance variable is the angle the PERPENDICULAR FACING THE IN FRONT (the side with the april tag) makes with the positive x axis (going towards the red alliance stations)
 */
public enum Into_The_Deep_2024_2025  implements PositionedAprilTag  {
    ONE(1,30,0,0,0),
    TWO(2,30,0,0,0),
    THREE(3,30,0,0,0),
    FOUR(4,30,0,0,0),
    FIVE(5,30,0,0,0),
    SIX(6,30,0,0,0),
    SEVEN(7,0,0,0,0),
    EIGHT(8,0,0,0,0),
    NINE(9,0,0,0,0),
    TEN(10,0,0,0,0);

    final double id, roll, xPos, yPos, heading;

    Into_The_Deep_2024_2025(double id,double roll,double xPos,double yPos,double heading){
        this.id=id;
        this.roll=roll;
        this.xPos=xPos;
        this.yPos=yPos;
        this.heading=heading;
    }

    public static Into_The_Deep_2024_2025 getAprilTag(int tag){
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


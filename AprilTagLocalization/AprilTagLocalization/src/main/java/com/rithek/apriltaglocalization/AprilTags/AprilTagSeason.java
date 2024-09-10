package com.rithek.apriltaglocalization.AprilTags;

public class AprilTagSeason {
    private static AprilTagSeason singleton=null;

    public static AprilTagSeason getInstance(){
        if (singleton!=null){
            return  singleton;
        }
        singleton = new AprilTagSeason();
        return singleton;
    }



    public AprilTagSeason(){

    }

}

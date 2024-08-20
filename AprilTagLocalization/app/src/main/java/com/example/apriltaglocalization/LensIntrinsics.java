package com.example.apriltaglocalization;

public class LensIntrinsics {
    float fx;
    float fy;
    float cx;
    float cy;

    public LensIntrinsics(float fx,float fy,float cx,float cy){
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }
}

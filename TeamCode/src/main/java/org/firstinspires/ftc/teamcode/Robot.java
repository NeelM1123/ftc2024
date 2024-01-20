package org.firstinspires.ftc.teamcode;
public class Robot {
    public Robot(){
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");


    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Chassis {
    protected final DcMotor leftDrive;
    protected final DcMotor rightDrive;
    int ticks_per_inch;

    public Chassis(DcMotor left_drive, DcMotor right_drive, int _ticks_per_inch) {
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive = left_drive;
        rightDrive = right_drive;
        ticks_per_inch=_ticks_per_inch;


    }

    public void move(double drive, double turn, boolean slow) {
        double max = 0.7;
        if (slow)
            max = 0.4;
        double leftPower = Range.clip(drive + turn, -max, max);
        double rightPower = Range.clip(drive - turn, -max, max);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void move(double distance ,double speed,double angle){
       double  xspeed = Range.clip(speed,0.0,1.0);
       double  xangle = Range.clip(angle,0.0,1.0);
       double xdistance= Range.clip(distance,0,150);
       int xticks = (int)(ticks_per_inch*xdistance);
       leftDrive.setPower(0);
       rightDrive.setPower(0);
       leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       leftDrive.setPower(xspeed);
       rightDrive.setPower(xspeed);
       leftDrive.setTargetPosition(xticks);
       rightDrive.setTargetPosition(xticks);
       leftDrive.setPower(0);
       rightDrive.setPower(0);
       leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
}

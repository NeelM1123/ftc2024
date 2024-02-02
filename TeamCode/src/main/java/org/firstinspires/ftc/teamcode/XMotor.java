package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
public class XMotor implements Runnable {
    protected DcMotor motor;

    protected double power;
    private int sleepTime=10,monitorInterval=500;
    private double tpower=0.0;


    public XMotor(DcMotor dc_motor){
        this.motor=dc_motor;
        this.tpower=0;
    }


    @Override
    public void run() {
        this.updateSpeed();

        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }
    protected void setPower(double power){
        this.motor.setPower(power);
    }

    private void updateSpeed(){

    }
    private void setXPower(){

    }
}

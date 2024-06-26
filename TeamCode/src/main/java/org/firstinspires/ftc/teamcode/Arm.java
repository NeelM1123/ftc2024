package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class Arm {
    public enum ARM_POSITION{
        BASIC_UP,
        READY_TO_PICK,
        LEVEL1,
        LEVEL2,
        LEVEL3
    }
   // boolean isArmBusy()
    protected DcMotorEx motor_lower_arm, motor_upper_arm;
    protected LinearOpMode lopmode;
    double len_lower_arm, len_upper_arm,  gear_ratio_lower_arm, gear_ratio_upper_arm, offset_ground, gear_ratio_lower_to_upper;
    double maxLowerArmPower=0.5;
    double maxUpperArmPower=0.4;
    ArmPosition basic,ready_pick,level1,level2;
    int initial_to_basic_position_ticks_lowerArm=414;
    int initial_to_basic_position_ticks_upperArm=-680;
    int ready_pick_lower_arm=97;
    int ready_pick_upper_arm=-680;
    int leve11_lower_arm=385;
    int level1_upper_arm=-680;
    int level2_lower_arm=135;
    int level2_upper_arm=-522;
    ArmPosition armPosLevel1, armPosLevel2, armPosLevel3,armPosBasic,armPosReadyPick;
    public Arm(DcMotorEx lower_arm_motor, DcMotorEx upper_arm_motor, double lower_arm_length, double upper_arm_length, double lower_arm_gear_ratio, double upper_arm_gear_ratio, double ground_offset, LinearOpMode lo){
        motor_lower_arm=lower_arm_motor;
        motor_upper_arm=upper_arm_motor;
        len_lower_arm=lower_arm_length;
        len_upper_arm=upper_arm_length;
        gear_ratio_lower_arm=lower_arm_gear_ratio;
        gear_ratio_upper_arm=upper_arm_gear_ratio;
        offset_ground=ground_offset;
        gear_ratio_lower_to_upper=gear_ratio_lower_arm/gear_ratio_lower_to_upper;
        initArmPositions();
        this.lopmode=lo;
        //init();
    }

    private void initArmPositions(){
        armPosBasic=new ArmPosition(414,-740);
        armPosReadyPick=new ArmPosition(97,-680);
        armPosLevel1 =new ArmPosition(385,-680);
        armPosLevel2 =new ArmPosition(135,-522);
        armPosLevel3=new ArmPosition(135,-450);
        motor_lower_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_upper_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lower_arm.setTargetPosition(0);
        motor_upper_arm.setTargetPosition(0);
        motor_lower_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_upper_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void init(){
        //motor_lower_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor_upper_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //this.moveX();

        moveToPosition(ARM_POSITION.BASIC_UP,true);

    }
    public void moveX(){
        //motor_lower_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor_lower_arm.setVelocity(0.0);
        this.motor_lower_arm.setTargetPosition(-200);
        motor_upper_arm.setTargetPosition(-650);

        this.motor_lower_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor_upper_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.motor_lower_arm.setVelocity(-400);
        lopmode.sleep(550);
        motor_upper_arm.setVelocity(-200);
        motor_lower_arm.setVelocity(150);
        motor_lower_arm.setTargetPosition(400);


        //while (this.motor_lower_arm.isBusy())
        //lopmode.sleep(2000);
        //this.motor_lower_arm.setPower(-0.5);
    }
    public void moveToPosition(ARM_POSITION arm_position, boolean resetEncoders){
        if (arm_position==ARM_POSITION.BASIC_UP)
            moveTo(armPosBasic);
            //setArmPosition(initial_to_basic_position_ticks_lowerArm, initial_to_basic_position_ticks_upperArm,resetEncoders);
        else if (arm_position==ARM_POSITION.READY_TO_PICK)
            moveTo(armPosReadyPick);
            //setArmPosition(ready_pick_lower_arm,ready_pick_upper_arm,resetEncoders);
        else if (arm_position==ARM_POSITION.LEVEL1)
            moveTo(armPosLevel1);
            //setArmPosition(leve11_lower_arm,level1_upper_arm,resetEncoders);
        else if (arm_position==ARM_POSITION.LEVEL2)
            moveTo(armPosLevel2);
            //setArmPosition(level2_lower_arm,level2_upper_arm,resetEncoders);
    }
    private void moveTo(ArmPosition armPos){

        int cposLower, cposUpper;
        cposLower=motor_lower_arm.getCurrentPosition();
        cposUpper=motor_upper_arm.getCurrentPosition();
        if (cposLower>-10 && cposUpper>-10){
            motor_lower_arm.setTargetPosition(70);
            motor_lower_arm.setVelocity(500);
            try {
                sleep(1250);
                motor_lower_arm.setVelocity(0);

            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        motor_upper_arm.setTargetPosition(armPos.upperArmEncoderValue);
        motor_lower_arm.setTargetPosition(armPos.lowerArmEncoderValue);
        motor_upper_arm.setVelocity(450);
        motor_lower_arm.setVelocity(590);
    }
    private void setArmPosition(int lower_arm_position,int upper_arm_position,boolean resetEncoders)  {
        DcMotor.RunMode la_mode=motor_lower_arm.getMode();
        DcMotor.RunMode ua_mode=motor_upper_arm.getMode();
        int step=15;
        int xpos_la,xstep_la;
        motor_lower_arm.setPower(0);
        motor_upper_arm.setPower(0);
        motor_upper_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_lower_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor_upper_arm.setPower(maxUpperArmPower);
        motor_lower_arm.setPower(maxLowerArmPower*.7);
        motor_upper_arm.setTargetPosition(upper_arm_position);
        lopmode.sleep(2500);
        motor_lower_arm.setTargetPosition(lower_arm_position);
/*
        for(int i=0;i<100;i++){
            if (abs(motor_upper_arm.getCurrentPosition() - upper_arm_position) > step)
                if (upper_arm_position < motor_upper_arm.getCurrentPosition())
                    motor_upper_arm.setTargetPosition(motor_upper_arm.getCurrentPosition() - step);
                else
                    motor_upper_arm.setTargetPosition(motor_upper_arm.getCurrentPosition() + step);
            lopmode.sleep(250);

            if (abs(motor_lower_arm.getCurrentPosition() - lower_arm_position) > step)
                if (lower_arm_position < motor_lower_arm.getCurrentPosition())
                    motor_lower_arm.setTargetPosition(motor_lower_arm.getCurrentPosition() - step);
                else
                    motor_lower_arm.setTargetPosition(motor_lower_arm.getCurrentPosition() + step);
        }


        motor_lower_arm.setTargetPosition(lower_arm_position);



            sleep(150);


        for (int i = 1 ;i<10;i++){

            if (!(motor_lower_arm.isBusy() || motor_upper_arm.isBusy()))
                break;
            try {
                sleep(50);
            }catch(InterruptedException ie){

            }
        }
        motor_lower_arm.setPower(0);
        motor_upper_arm.setPower(0);
        if (resetEncoders){
            motor_lower_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_upper_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motor_lower_arm.setMode(la_mode);
        motor_upper_arm.setMode(ua_mode);
*/

    }

    public void moveHorizontal(double powerFactor){
        double lowerArmMotorPower=Range.clip(powerFactor, -1.0, 1.0);
        double upperArmMotorPower=-lowerArmMotorPower/gear_ratio_lower_to_upper;
        motor_upper_arm.setPower(upperArmMotorPower);
    }
    public void moveUpperArm(double powerFactor){
        moveArmPart(motor_upper_arm,powerFactor);
    }
    public void moveLowerArm(double powerFactor){
        moveArmPart(motor_lower_arm,powerFactor);
    }
    private void moveArmPart(DcMotor armPart,double powerFactor){
        if (armPart.getMode()==DcMotor.RunMode.RUN_TO_POSITION){
            armPart.setPower(0);
            armPart.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        armPart.setPower(Range.clip(powerFactor,-1.0*maxLowerArmPower,1.0*maxLowerArmPower));

    }

}

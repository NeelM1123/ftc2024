package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    protected DcMotor motor_lower_arm, motor_upper_arm;
    double len_lower_arm, len_upper_arm,  gear_ratio_lower_arm, gear_ratio_upper_arm, offset_ground, gear_ratio_lower_to_upper;
    double maxLowerArmPower=0.5;
    double maxUpperArmPower=0.4;
    int initial_to_basic_position_ticks_lowerArm=414;
    int initial_to_basic_position_ticks_upperArm=-732;
    public Arm(DcMotor lower_arm_motor, DcMotor upper_arm_motor, double lower_arm_length,double upper_arm_length, double lower_arm_gear_ratio,double upper_arm_gear_ratio,double ground_offset){
        motor_lower_arm=lower_arm_motor;
        motor_upper_arm=upper_arm_motor;
        len_lower_arm=lower_arm_length;
        len_upper_arm=upper_arm_length;
        gear_ratio_lower_arm=lower_arm_gear_ratio;
        gear_ratio_upper_arm=upper_arm_gear_ratio;
        offset_ground=ground_offset;
        gear_ratio_lower_to_upper=gear_ratio_lower_arm/gear_ratio_lower_to_upper;
        init();
    }

    public void init(){
        motor_lower_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_upper_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveToPosition(ARM_POSITION.BASIC_UP,true);

    }
    public void moveToPosition(ARM_POSITION arm_position, boolean resetEncoders){
        if (arm_position==ARM_POSITION.BASIC_UP)
            setArmPosition(initial_to_basic_position_ticks_lowerArm, initial_to_basic_position_ticks_upperArm,resetEncoders);


    }

    private void setArmPosition(int lower_arm_position,int upper_arm_position,boolean resetEncoders)  {
        DcMotor.RunMode la_mode=motor_lower_arm.getMode();
        DcMotor.RunMode ua_mode=motor_upper_arm.getMode();
        motor_lower_arm.setPower(0);
        motor_upper_arm.setPower(0);
        motor_lower_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_upper_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_upper_arm.setTargetPosition(upper_arm_position);

        motor_lower_arm.setTargetPosition(lower_arm_position);
        motor_upper_arm.setPower(maxUpperArmPower/2);

        try{
            sleep(550);}catch(InterruptedException ie){}
        motor_lower_arm.setPower(maxLowerArmPower/2);

        /*
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

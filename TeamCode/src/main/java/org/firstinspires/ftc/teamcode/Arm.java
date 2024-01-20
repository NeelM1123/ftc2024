package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Arm {
    private DcMotor motor_lower_arm, motor_upper_arm;
    double len_lower_arm, len_upper_arm,  gear_ratio_lower_arm, gear_ratio_upper_arm, offset_ground, gear_ratio_lower_to_upper;
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

    }

    public void moveHorizontal(double powerFactor){
        double lowerArmMotorPower=Range.clip(powerFactor, -1.0, 1.0);
        double upperArmMotorPower=-lowerArmMotorPower/gear_ratio_lower_to_upper;
        motor_upper_arm.setPower(upperArmMotorPower);
    }
    public void moveUpperArm(double powerFactor){
        motor_upper_arm.setPower(Range.clip(powerFactor, -1.0, 1.0));
    }
    public void moveLowerArm(double powerFactor){
        motor_lower_arm.setPower(Range.clip(powerFactor, -1.0, 1.0));
    }

}

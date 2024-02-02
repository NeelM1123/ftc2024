package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Robot {
    protected Chassis chassis;
    protected Flight flight;
    protected Hand hand;
    protected Arm arm;

    public Robot(HardwareMap hardwareMap, LinearOpMode lo){
        chassis = new Chassis(hardwareMap.get(DcMotorEx.class, "left_drive"), hardwareMap.get(DcMotorEx.class, "right_drive"),10);
        arm = new Arm(hardwareMap.get(DcMotorEx.class, "motor1"),
                hardwareMap.get(DcMotorEx.class, "motor2"),
                7.0,
                13.0,
                0.25, 0.5, 1.5,lo);
        hand = new Hand(hardwareMap.get(Servo.class, "servo1"), hardwareMap.get(Servo.class, "servo"));
        flight = new Flight(hardwareMap.get(Servo.class, "servo2"));
    }
    public Robot(Chassis _chassis, Arm _arm, Hand _hand, Flight _flight){
        chassis=_chassis;
        arm=_arm;
        hand=_hand;
        flight=_flight;
    }

    public void gamepadControl(Gamepad controllerA, Gamepad controllerB){
            if (controllerA.back){
                if (controllerA.right_bumper)
                    flight.launch();

                return;
            }
            if (controllerB.left_bumper){
                if (controllerB.y)
                    arm.moveToPosition(Arm.ARM_POSITION.READY_TO_PICK,false);
                else if (controllerB.b)
                    arm.moveToPosition(Arm.ARM_POSITION.LEVEL1,false);
                else if (controllerB.a)
                    arm.moveToPosition(Arm.ARM_POSITION.LEVEL2,false);
                else if (controllerB.x)
                    arm.moveToPosition(Arm.ARM_POSITION.BASIC_UP,false);

                return;
            }
            if(controllerA.left_bumper)
                chassis.move(controllerA.left_stick_y,-controllerA.right_stick_x, true);
            else
                chassis.move(controllerA.left_stick_y,-controllerA.right_stick_x, false);

            //arm.moveLowerArm(controllerB.left_stick_y);
            //arm.moveUpperArm(controllerB.right_stick_y);
            if (controllerA.x ||controllerB.x){
                //arm.moveX();
                hand.moveFingers(Hand.FingersPosition.flex);
            }
            if (controllerA.y || controllerB.y){
                hand.moveFingers(Hand.FingersPosition.extend);
            }
            if (controllerA.dpad_down || controllerB.dpad_down){
                hand.moveWrist(Hand.WristPosition.down);
            }
            if (controllerA.dpad_up||controllerB.dpad_up){
                hand.moveWrist(Hand.WristPosition.up);
            }

    }





}

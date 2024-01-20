package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Robot {
    protected Chassis chassis;
    private Flight flight;
    private Hand hand;
    private Arm arm;

    public Robot(HardwareMap hardwareMap){
        chassis = new Chassis(hardwareMap.get(DcMotor.class, "left_drive"), hardwareMap.get(DcMotor.class, "right_drive"),10);
        arm = new Arm(hardwareMap.get(DcMotor.class, "motor1"),
                hardwareMap.get(DcMotor.class, "motor2"),
                7.0,
                13.0,
                0.25, 0.5, 1.5);
        hand = new Hand(hardwareMap.get(Servo.class, "servo"), hardwareMap.get(Servo.class, "servo1"));
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
            }
            chassis.move(controllerA.left_stick_y,-controllerA.right_stick_x);
            arm.moveLowerArm(controllerB.left_stick_y);
            arm.moveUpperArm(controllerB.right_stick_y);
            if (controllerA.b){
                hand.moveFingers(Hand.FingersPosition.flex);
            }
            if (controllerA.y){
                hand.moveFingers(Hand.FingersPosition.extend);
            }
            if (controllerA.dpad_down){
                hand.moveWrist(Hand.WristPosition.down);
            }
            if (controllerA.dpad_up){
                hand.moveWrist(Hand.WristPosition.up);
            }

    }





}

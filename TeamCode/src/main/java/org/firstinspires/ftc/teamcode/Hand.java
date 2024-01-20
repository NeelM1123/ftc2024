package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Hand {
    private final Servo servo_hand;
    private final Servo servo_wrist;

    public Hand(Servo hand_servo, Servo wrist_servo) {
        servo_hand = hand_servo;
        servo_wrist = wrist_servo;
        init();
    }

    public void init() {
        servo_hand.setPosition(0.0);
        servo_wrist.setPosition(0.0);
    }

    public void moveFingers(FingersPosition to_fingersPosition) {
        if (to_fingersPosition == FingersPosition.flex) {
            servo_hand.setPosition(1.0);
        } else {
            servo_hand.setPosition(0.0);
        }
    }

    public void moveWrist(WristPosition to_wristPosition) {
        if (to_wristPosition == WristPosition.up) {
            servo_wrist.setPosition(1.0);
        } else {
            servo_wrist.setPosition(0.0);
        }

    }

    public enum FingersPosition {flex, extend}

    public enum WristPosition {up, down}


}

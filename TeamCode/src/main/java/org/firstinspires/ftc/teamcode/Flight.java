package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Flight {
    private final Servo servo_launcher;

    public Flight(Servo launcher_servo) {
        servo_launcher = launcher_servo;
    }

    public void init() {
        servo_launcher.setPosition(0.0);
    }

    public void launch() {
        servo_launcher.setDirection(Servo.Direction.FORWARD);
        servo_launcher.setPosition(1.0);
        //todo
    }

}
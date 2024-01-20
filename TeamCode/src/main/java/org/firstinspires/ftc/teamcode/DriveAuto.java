package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Linear OpMode DriveAutonomous", group = "Linear OpMode")
public class DriveAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap,this);
        waitForStart();

        while (opModeIsActive()) {
            robot.chassis.move(36,0.3,0);
            sleep(10);
        }
    }
/*
    private void setRobotAssembly(){
        Chassis chassis = new Chassis(hardwareMap.get(DcMotor.class, "left_drive"), hardwareMap.get(DcMotor.class, "right_drive"));
        Arm arm = new Arm(hardwareMap.get(DcMotor.class, "motor1"),
                hardwareMap.get(DcMotor.class, "motor2"),
                7.0,
                13.0,
                0.25, 0.5, 1.5);
        Hand hand = new Hand(hardwareMap.get(Servo.class, "servo"), hardwareMap.get(Servo.class, "servo"));
        Flight flight = new Flight(hardwareMap.get(Servo.class, "servo"));
        robot=new Robot(chassis, arm, hand, flight);

    }
    */
}

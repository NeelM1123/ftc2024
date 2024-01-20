package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Linear OpMode DriveTeleOp", group = "Linear OpMode")
public class DriveTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            robot.gamepadControl(gamepad1,gamepad2);
            telemetry.addData("Pos lowerArm: ",robot.arm.motor_lower_arm.getCurrentPosition());
            telemetry.addData("Pos upperArm: ",robot.arm.motor_upper_arm.getCurrentPosition());
            telemetry.addData("Pos leftDrive: ",robot.chassis.leftDrive.getCurrentPosition());
            telemetry.addData("Pos rightDrive: ",robot.chassis.rightDrive.getCurrentPosition());
            telemetry.addData("Pos hand: ",robot.hand.servo_hand.getPosition());
            telemetry.addData("Pos wrist: ",robot.hand.servo_wrist.getPosition());
            telemetry.addData("la mode:", robot.arm.motor_lower_arm.getMode().toString());
            telemetry.update();





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

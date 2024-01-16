package org.firstinspires.ftc.teamcode;/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Linear OpMode Test", group="Linear OpMode")
public class test extends LinearOpMode {

    // Declare OpMode members for each of the 2 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private  DcMotor leftDrive,rightDrive = null;
    private Servo servo = null;
    private Servo servo1 = null;
    private Servo servo2 = null;

    double leftPower;
    double rightPower;

    // run until the end of the match (driver presses STOP)
    double power1=1.0;
    double power2=1.0;
    double damp=0.7;
    double ratio=0.9;

    double finalPower4 = power1;
    double finalPower5 = power2;

    boolean isConfigMode=false;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        servo = hardwareMap.get(Servo.class, "servo");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");


        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //power2=power1*0.6;
        power1=power1*damp;
        power2=power2*damp;



        servo.setPosition(0);
        while (opModeIsActive()) {
            if (gamepad1.back){
                isConfigMode=!isConfigMode;
                //telemetry.addLine("ConfigMode: " + isConfigMode);
                telemetry.addData("Status", "ConfigMode: " + isConfigMode);
                telemetry.update();
            }
            if (isConfigMode){
                adjustConfig();
            }else{
                controlRobot();
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }}
    public void adjustConfig(){
        boolean updated=false;
        if (gamepad1.dpad_up){
            damp+=0.1;
            if (damp>1)
                damp=1.0;
            updated=true;
        } else if (gamepad1.dpad_down) {
            damp-=0.1;
            if (damp<0.2)
                damp=0.2;
            updated=true;
        } else if (gamepad1.dpad_left) {
            ratio-=0.1;
            if (ratio<0.1)
                ratio=0.1;
            updated=true;
        } else if (gamepad1.dpad_right) {
            ratio+=0.1;
            if (ratio>2)
                ratio=2;
            updated=true;
        }
        if (updated){
            telemetry.addData("Status","Config:" + "Damp: " + damp + " Ratio: " + ratio);
            telemetry.update();
        }

    }
    public void controlRobot(){

    }

    public void xd(){

        class CombinedArmsUp {
            void execute() {
                while (opModeIsActive() && gamepad1.left_bumper) {
                    motor1.setDirection(DcMotor.Direction.FORWARD);
                    motor2.setDirection(DcMotor.Direction.REVERSE);
                    motor1.setPower(finalPower4);
                    motor2.setPower(-finalPower5 * ratio);
                    sleep(10);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }

        // Combined Arms Down
        double finalPower6 = power1;
        double finalPower7 = power2;
        class CombinedArmsDown {
            void execute() {
                while (opModeIsActive() && gamepad1.right_bumper) {
                    motor1.setDirection(DcMotor.Direction.FORWARD);
                    motor2.setDirection(DcMotor.Direction.REVERSE);
                    motor1.setPower(-finalPower6);
                    motor2.setPower(finalPower7 * ratio);
                    sleep(10);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }

            /*while (gamepad1.left_bumper == true) {
                motor1.setDirection(DcMotor.Direction.FORWARD);
                motor2.setDirection(DcMotor.Direction.REVERSE);
                motor1.setPower(power1);
                motor2.setPower(-power2*ratio);
                sleep(10);
                motor1.setPower(0);
                motor2.setPower(0);
                motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            while (gamepad1.right_bumper){
                motor1.setDirection(DcMotor.Direction.FORWARD);
                motor2.setDirection(DcMotor.Direction.REVERSE);
                motor1.setPower(-power1);
                motor2.setPower(power2*ratio);
                sleep(10);
                motor1.setPower(0);
                motor2.setPower(0);
                motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }*/
        //        while (gamepad1.x) {
        //            motor1.setDirection(DcMotor.Direction.FORWARD);
        //            motor1.setPower(power1);
        //            sleep(10);
        //            motor1.setPower(0);
        //            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //        }

        //        while (gamepad1.y) {
        //            motor1.setDirection(DcMotor.Direction.FORWARD);
        //            motor1.setPower(-power1);
        //            sleep(10);
        //            motor1.setPower(0);
        //            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //        }
        double finalPower1 = power1;
        class Arm1Forward {
            void execute() {
                while (opModeIsActive() && gamepad1.left_trigger > 0.5) {
                    motor1.setDirection(DcMotor.Direction.FORWARD);
                    motor1.setPower(finalPower1);
                    sleep(10);
                    motor1.setPower(0);
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }

        double finalPower = power1;
        class Arm1Backward {
            void execute() {
                while (opModeIsActive() && gamepad1.right_trigger > 0.5) {
                    motor1.setDirection(DcMotor.Direction.FORWARD);
                    motor1.setPower(-finalPower);
                    sleep(10);
                    motor1.setPower(0);
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }

        double finalPower2 = power2;
        class Arm2Forward {
            void execute() {
                while (opModeIsActive() && gamepad1.dpad_left) {
                    motor2.setDirection(DcMotor.Direction.FORWARD);
                    motor2.setPower(finalPower2);
                    sleep(10);
                    motor2.setPower(0);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }

        double finalPower3 = power2;
        class Arm2Backward {
            void execute() {
                while (opModeIsActive() && gamepad1.dpad_right) {
                    motor2.setDirection(DcMotor.Direction.FORWARD);
                    motor2.setPower(-finalPower3);
                    sleep(10);
                    motor2.setPower(0);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }










            /*while (gamepad1.left_trigger>0.5) {
                motor1.setDirection(DcMotor.Direction.FORWARD);
                motor1.setPower(power1);
                sleep(10);
                motor1.setPower(0);
                motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            while (gamepad1.right_trigger>0.5) {
                motor1.setDirection(DcMotor.Direction.FORWARD);
                motor1.setPower(-power1);
                sleep(10);
                motor1.setPower(0);
                motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            while (gamepad1.dpad_left) {
                motor2.setDirection(DcMotor.Direction.FORWARD);
                motor2.setPower(power2);
                sleep(10);
                motor2.setPower(0);
                motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
            while (gamepad1.dpad_right){
                motor2.setDirection(DcMotor.Direction.FORWARD);
                motor2.setPower(-power2);
                sleep(10);
                motor2.setPower(0);
                motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }*/

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        class AirplaneLaunch {
            void execute() {
                while (opModeIsActive() && gamepad1.y) {
                    servo2.setPosition(servo.getPosition() - 1);
                }
            }
        }

        class WristUp {
            void execute() {
                while (opModeIsActive() && gamepad1.dpad_up) {
                    servo1.setPosition(servo.getPosition() + 1);
                }
            }
        }

        class WristDown {
            void execute() {
                while (opModeIsActive() && gamepad1.dpad_down) {
                    servo1.setPosition(servo.getPosition() - 1);
                }
            }
        }

        class Flex {
            void execute() {
                while (opModeIsActive() && gamepad1.a) {
                    servo.setPosition(servo.getPosition() + 1);
                }
            }
        }

        class Extend {
            void execute() {
                while (opModeIsActive() && gamepad1.b) {
                    servo.setPosition(servo.getPosition() - 1);
                }
            }
        }



            /*if (gamepad1.a) {
                servo.setPosition(servo.getPosition()+1);
            }


            if (gamepad1.b) {
                servo.setPosition(servo.getPosition()-1);
            }




            if (gamepad1.dpad_up) {
                servo1.setPosition(servo.getPosition()+1);
            }


            if (gamepad1.dpad_down) {
                servo1.setPosition(servo.getPosition()-1);
            }





            if (gamepad1.x) {
                servo2.setPosition(servo.getPosition()+1);
            }


            if (gamepad1.y) {
                servo2.setPosition(servo.getPosition()-1);
            }*/



    }
}

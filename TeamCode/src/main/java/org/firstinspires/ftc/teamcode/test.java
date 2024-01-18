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

import static java.lang.Math.abs;

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
    double ratio=0.5;
    double xratio=0.0;
    double finalPower4 = power1;
    double finalPower5 = power2;

    boolean isConfigMode=false;
    int pos_m1,pos_m2;
    //double pos_m2=0;

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

        int mpos_1,mpos_2,mpos_3,mpos_4;

        servo.setPosition(0);

        while (opModeIsActive()) {
            if (gamepad1.back){
                isConfigMode=!isConfigMode;
                //telemetry.addLine("ConfigMode: " + isConfigMode);
                //telemetry.update();
                sleep(300);
            }
            if (isConfigMode){
                adjustConfig();
                sleep(100);
            }else{
                controlRobot();
            }



            // Show the elapsed game time and wheel power.
            mpos_1=motor1.getCurrentPosition();
            mpos_2= motor2.getCurrentPosition();
            mpos_3=leftDrive.getCurrentPosition();
            mpos_4=rightDrive.getCurrentPosition();

            telemetry.addData("ConfigMode",  isConfigMode);
            telemetry.addData("Config","Damp: " + damp + " Ratio: " + ratio);
            telemetry.addData("MPos 1",mpos_1);
            telemetry.addData("MPos 2",mpos_2);
            telemetry.addData("MPos 3",mpos_3);
            telemetry.addData("MPos 4",mpos_4);
            telemetry.addData("xratio",xratio);




            telemetry.addData("Runtime",  runtime.toString());
            telemetry.update();
            sleep(10);

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
       // if (updated){
            //telemetry.update();
       // }

    }
    public void initArm(){
        motor1.setTargetPosition(pos_m1);
        motor2.setTargetPosition(pos_m2);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(power1);
        motor2.setPower(power1);

    }
    public void controlRobot(){
        boolean moveArm = false;
        moveArm= gamepad1.right_bumper || gamepad1.left_bumper || gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y;

        moveRobot();
        moveArm();
        /*
        if (!moveArm){
            motor1.setPower(0);
            motor2.setPower(0);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

         */
    }
    public void moveArm() {
        if (!(gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b))
            return;
        if (gamepad1.left_bumper || gamepad1.right_bumper)
            moveArmX();
        else
            moveArmY();
    }
    public void moveArmX() {
        double txratio=2.0;
        double step=5;
        pos_m1=(int)(motor1.getCurrentPosition()+txratio*step);
        pos_m2=(int)(motor2.getCurrentPosition()+step)
        /*
        double power1 = damp;
        double power2 = -power1 * ratio;
        int mpos1, mpos2, mpos1_new, mpos2_new, delta_mpos1, delta_mpos2;
        mpos1 = motor1.getCurrentPosition();
        mpos2 = motor2.getCurrentPosition();
        int tdelta = 40;
        double txratio=2.0;
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(power1);
        motor2.setPower(power1);
        if (gamepad1.left_bumper){
            motor1.setTargetPosition((int) (mpos1+(txratio*tdelta)));
            motor2.setTargetPosition(mpos2-tdelta);
        }else if (gamepad1.right_bumper){
            motor1.setTargetPosition((int)(mpos1-txratio*tdelta));
            motor2.setTargetPosition(mpos2+tdelta);
        }
        */
         */
        /*

        for (int i=0;i<100;i++){
            if (!(motor1.isBusy() || motor2.isBusy()) )
                break;
            sleep(10);
        }

         */
        //sleep(50);
    }
    public void moveArmY() {
        int step=5;

        //motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (gamepad1.x)
            pos_m1=motor1.getCurrentPosition()+step;
            //motor1.setPower(power1);
        if (gamepad1.b)
            pos_m1=motor1.getCurrentPosition()-step;

        //motor1.setPower(-power1);
        if (gamepad1.a)
            pos_m1=motor2.getCurrentPosition()-step;

        //motor2.setPower(-power1);
        if (gamepad1.y)
            pos_m1=motor1.getCurrentPosition()+step;

        // motor2.setPower(power1);

    }

    public void moveArm2() {
        if (!(gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b))
            return;

        double power1 = damp;
        double power2 = -power1 * ratio;
        double mpos1, mpos2, mpos1_new, mpos2_new, delta_mpos1, delta_mpos2;
        mpos1 = motor1.getCurrentPosition();
        mpos2 = motor2.getCurrentPosition();
        if (gamepad1.left_bumper || gamepad1.right_bumper){
            if (gamepad1.left_bumper) {
                motor1.setPower(power1);
                motor2.setPower(power2);
            } else if (gamepad1.right_bumper) {
                motor1.setPower(-power1);
                motor2.setPower(-power2);
            }
            sleep(10);
            delta_mpos2=motor2.getCurrentPosition()-mpos2;
            delta_mpos1=motor1.getCurrentPosition()-mpos1;
            xratio=abs(delta_mpos1/delta_mpos2+0.5);
                if (xratio > 2.05) {
                    ratio += 0.1;
                } else if (xratio < 1.95)
                    ratio -= 0.1;
            for(int i =0 ;i<10;i++) {
                delta_mpos2=motor2.getCurrentPosition()-mpos2;
                //delta_mpos1=motor1.getCurrentPosition()-mpos1;
                if (xratio<2.05 && xratio>1.95){
                    break;
                }else if(xratio>2.05){
                    int j=10;
                    //motor2.setTargetPosition();
                }
                xratio=abs(delta_mpos1/delta_mpos2+0.005);
            }




        } else {
            if (gamepad1.x)
                motor1.setPower(power1);
            if (gamepad1.b)
                motor1.setPower(-power1);
            if (gamepad1.a)
                motor2.setPower(-power1);
            if (gamepad1.y)
                motor2.setPower(power1);


     //       sleep(10);
        }

    }
    public void moveRobot(){
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
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

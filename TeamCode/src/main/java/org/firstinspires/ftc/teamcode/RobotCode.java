/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @b*/
@TeleOp(name="Robot Code", group="Linear Opmode")
//@Disabled
public class RobotCode <clawControl> extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armControl = null;
    private DcMotor armControl2 = null;
    private Servo clawControl = null;
//    private Servo clawControl2 = null;
    private Servo armPosition = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armControl = hardwareMap.get(DcMotor.class, "arm_control");
        armControl2 = hardwareMap.get(DcMotor.class, "arm_control2");
        clawControl = hardwareMap.get(Servo.class, "claw_control");
        //clawControl2 = hardwareMap.get(Servo.class, "claw_control");
        armPosition = hardwareMap.get(Servo.class, "arm_position");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        clawControl.setDirection(Servo.Direction.FORWARD);
//        clawControl2.setDirection(Servo.Direction.REVERSE);
        clawControl.setPosition(.8);
        telemetry.addLine("Test");
//        armPosition = Range.
        armPosition.setPosition(0.79);
//        armControl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armControl2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armControl2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        System.out.println(armPosition)
//        armPosition.setDirection(Servo.Direction.FORWARD);

        //wait to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //boolean dpadFalse = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double armPower;
            double armPower2;
            double clawPower;
            double clawPower2;
            double speed;
            //double armPreset;
            //double current = clawControl.getPosition();

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


            // Tank Mode uses one stick to control each wheel.4
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);


//            if (gamepad1.dpad_up)
//            {
//                if (dpadFalse)
//                {
//                    leftPower = gamepad1.left_stick_y + 50;
//                    rightPower = gamepad1.right_stick_y + 50;
//                }
//
//                dpadFalse =! dpadFalse;
//
//            }
//
//            else if (gamepad1.dpad_down)
//            {
//                if (dpadFalse)
//                {
//                    leftPower = gamepad1.left_stick_y - 50;
//                    rightPower = gamepad1.right_stick_y - 50;
//                }
//
//                dpadFalse =! dpadFalse;
//            }

            armPower = gamepad2.right_stick_y;

            armControl.setPower(armPower/3);
            armControl2.setPower(armControl.getPower());

            if(gamepad2.left_stick_y > 0)
            {
                armPosition.setPosition(armPosition.getPosition()+0.0006);

            }
            else if(gamepad2.left_stick_y < 0)
            {
                armPosition.setPosition(armPosition.getPosition()-0.0006);
            }

//            armPower2 = gamepad2.right_stick_y;
//            armControl2.setPower(armPower2);

//            clawPower = gamepad2.left_stick_x;
//            clawControl.setPosition(clawPower);
            if(gamepad2.right_bumper)
            {
//                double adding = 0.25d;
                clawControl.setPosition(clawControl.getPosition()+0.0006);
//                System.out.println("clawControl: " + clawControl.getPosition());
            }

            if(gamepad2.left_bumper)
            {
//                double incrementalChange = -0.25;
                clawControl.setPosition(clawControl.getPosition()-0.0006);
//                System.out.println("clawControl: " + clawControl.getPosition());
            }

            if(gamepad2.right_trigger > 0)
            {
                clawControl.setPosition(.8);
            }

            if(gamepad2.left_trigger > 0)
            {
                clawControl.setPosition(0.67);
            }

////            boolean dTrue = gamepad2.a; This is not part of the working code
//
//            if(gamepad2.a)
//            {
//                if(armControl.getCurrentPosition() < -75)
//                {
//                    armControl.setDirection(DcMotorSimple.Direction.FORWARD);
//                    armControl2.setDirection(armControl.getDirection());
//                }
//                else if(armControl.getCurrentPosition() > -75)
//                {
//                    armControl.setDirection(DcMotorSimple.Direction.REVERSE);
//                    armControl2.setDirection(armControl.getDirection());
//                }
//                armPosition.setPosition(0.48);
//
//                armControl.setTargetPosition(-75);
//                armControl2.setTargetPosition(armControl.getTargetPosition());
//
//                armControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armControl2.setMode(armControl.getMode());
//
//                armControl.setPower(0.3);
//                armControl2.setPower(armControl.getPower());
//
//                while (opModeIsActive() && (armControl.isBusy() || armControl2.isBusy()))
//                {
//
//                }
//
//                armControl.setPower(0);
//                armControl2.setPower(armControl.getPower());
//
//                armControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                armControl2.setMode(armControl.getMode());
//            }
//
//            armControl.setDirection(DcMotorSimple.Direction.FORWARD);
//            armControl2.setDirection(armControl.getDirection());
//
//            armControl.setPower(armPower/3);
//            armControl2.setPower(armControl.getPower());

            if(gamepad1.a)
            {
                telemetry.addData("Arm Position: ", armPosition.getPosition());
                //0.73
                telemetry.addData("Arm Control: ", armControl.getCurrentPosition());
                telemetry.addData("Arm Control2: ", armControl2.getCurrentPosition());
                //11
                telemetry.addData("leftDrive: ", leftDrive.getCurrentPosition());
                telemetry.addData("rightDrive: ", rightDrive.getCurrentPosition());
                telemetry.addData("Claw Control: ", clawControl.getPosition());
            }

//            if(gamepad1.b)
//            {
//                armControl.setTargetPosition(11);
//                armControl2.setTargetPosition(armControl.getCurrentPosition());
//                armControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armControl.setPower(2);
//                armControl2.setPower(armControl.getPower());
//            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
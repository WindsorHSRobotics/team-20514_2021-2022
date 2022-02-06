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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous_Blue", group="Pushbot")
//@Disabled
public class Autonomous_Blue extends Autonomous_Something {

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();



    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo armPosition = null;
    private Servo clawControl = null;

    @Override
    public void runOpMode() {
        /************************************Updated************************************/
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        clawControl = hardwareMap.get(Servo.class, "claw_control");
        armPosition = hardwareMap.get(Servo.class, "arm_position");



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        waitForStart();
        clawControl.setPosition(0.67);

        sleep(1000);

        armPosition.setPosition(0.17);

        sleep(1000);

        this.driveForward(1.5, "Leg 1: %2.5f S Elapsed", leftDrive, rightDrive);
        //original is 2
        clawControl.setPosition(0.9);
        this.driveBackward(0.3, "test", leftDrive, rightDrive);
        clawControl.setPosition(0.67);
        this.turn(0.74, "Leg 2: %2.5f S Elapsed",Direction.RIGHT, leftDrive, rightDrive);
        //original is 0.6, 0.6 is too little
        armPosition.setPosition(0.49);
        this.driveForward(  4.5, "Leg 3: %2.5f S Elapsed", leftDrive, rightDrive);
        //original is 3

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);


        /************************************Original***********************************/

//        /*
//         * Initialize the drive system variables.
//         * The init() method of the hardware class does all the work here
//         */
//        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//        armPosition = hardwareMap.get(Servo.class, "arm_position");
//
//        armPosition.setPosition(0.25);
//        // Send telemetry message to signify robot waiting;
//         telemetry.addData("Status", "Ready to run");    //
//         telemetry.update();
//
        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
//
//        // Step 1:  Drive forward for 1.5 seconds

//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);
//
//
//
//        telemetry.addData("Status", "Ready to run");    //
//        telemetry.update();
//
//        runtime.reset();
//
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            leftDrive.setPower(-FORWARD_SPEED/2);
//            rightDrive.setPower(FORWARD_SPEED/2);
//        }
//
//        // Step 2:  Spin right for 1.3 seconds

//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
//            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            leftDrive.setPower(TURN_SPEED);
//            rightDrive.setPower(TURN_SPEED);
//        }
//
//        // Step 3:  Drive Forward for 5 Seconds

//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2)) {
//            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            leftDrive.setPower(-FORWARD_SPEED/2);
//            rightDrive.setPower(FORWARD_SPEED/2);
//        }
//
//
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);
    }


}

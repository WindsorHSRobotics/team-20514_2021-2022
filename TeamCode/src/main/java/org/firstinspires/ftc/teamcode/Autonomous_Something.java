package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Autonomous_Something", group="Pushbot")
//@Disabled
public abstract class Autonomous_Something  extends LinearOpMode {
    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.8;
    static final double TURN_SPEED = 0.8;

//    private DcMotor leftDrive = null;
//    private DcMotor rightDrive = null;
//    private Servo armPosition = null;


    protected void turn(double duration, String logMessage, Direction dir, DcMotor leftDrive,
                        DcMotor rightDrive) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData("Path", logMessage, runtime.seconds());
            telemetry.update();
            if (dir.equals(Direction.LEFT)) {
                leftDrive.setPower(TURN_SPEED);
                rightDrive.setPower(TURN_SPEED);
            } else if (dir.equals(Direction.RIGHT)) {
                leftDrive.setPower(-TURN_SPEED);
                rightDrive.setPower(-TURN_SPEED);
            }
        }
    }

    protected void driveBackward(double duration, String logMessage, DcMotor leftDrive,
                                 DcMotor rightDrive) {
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


//        telemetry.addData("Status", "Ready to run");    //
//        telemetry.update();

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < duration)) {
            //telemetry.addData("Path", logMessage, runtime.seconds());
            telemetry.update();
            leftDrive.setPower(-FORWARD_SPEED / 2);
            rightDrive.setPower(FORWARD_SPEED / 2);
        }
    }

    protected void driveForward(double duration, String logMessage, DcMotor leftDrive,
                                DcMotor rightDrive) {
        // Step 1:  Drive forward for 1.5 seconds
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


//        telemetry.addData("Status", "Ready to run");
//        telemetry.update();

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData("Path", logMessage, runtime.seconds());
            telemetry.update();
            leftDrive.setPower(-FORWARD_SPEED / 2);
            rightDrive.setPower(FORWARD_SPEED / 2);
        }
    }
}


// protected abstract void driveForward(int i, String s, DcMotor leftDrive, DcMotor rightDrive);



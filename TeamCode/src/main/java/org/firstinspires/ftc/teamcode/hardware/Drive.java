package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive {
    LinearOpMode opMode;
    public DcMotor FL;
    public DcMotor BL;
    public DcMotor FR;
    public DcMotor BR;

    // Should move intake servo code to its own class.
    CRServo lServo;
    CRServo rServo;

    public static double TICKS_PER_CM = 17.5;
    public static double TICKS_PER_DEGREE = 10.72;
    int tolerance = 20;

    public Drive(LinearOpMode opModeCalledFrom) {
        opMode = opModeCalledFrom;
        FL = opMode.hardwareMap.get(DcMotor.class, "FL");
        BL = opMode.hardwareMap.get(DcMotor.class, "BL");
        FR = opMode.hardwareMap.get(DcMotor.class, "FR");
        BR = opMode.hardwareMap.get(DcMotor.class, "BR");

        // Move intake and outtake code to its own class
        rServo = opMode.hardwareMap.get(CRServo.class, "rServo");
        lServo = opMode.hardwareMap.get(CRServo.class, "lServo");


        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Move intake and outtake code to its own class
        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);
    }

    // Method that drives the robot positive or negative a given distance and speed in cm
    public void drive(double speed, double distance) {

        int ticks = (int) (distance * TICKS_PER_CM);

        if (opMode.opModeIsActive()) {

            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Reset encoder positions to 0
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set the motor's target positions
            FL.setTargetPosition(ticks);
            BL.setTargetPosition(ticks);
            FR.setTargetPosition(ticks);
            BR.setTargetPosition(ticks);

            // Turn on RUN_TO_POSITION mode
            // Set motors to run to the given target position
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set motor power to get motors moving
            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            // Wait until all motors are within tolerance of their target
            while ( opMode.opModeIsActive() && (
                    Math.abs(BL.getCurrentPosition() - ticks) > tolerance ||
                    Math.abs(BR.getCurrentPosition() - ticks) > tolerance ||
                    Math.abs(FL.getCurrentPosition() - ticks) > tolerance ||
                    Math.abs(FR.getCurrentPosition() - ticks) > tolerance
                    )
                ) {
                opMode.telemetry.addLine("IN LOOP");
                opMode.telemetry.addData("FL pos", FL.getCurrentPosition());
                opMode.telemetry.addData("BL pos", BL.getCurrentPosition());
                opMode.telemetry.addData("BR pos", BR.getCurrentPosition());
                opMode.telemetry.addData("FR pos", FR.getCurrentPosition());
                opMode.telemetry.update();
            }
            opMode.sleep(100);
            // Power motors off
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }
    }

    public void rotate(double speed, double degrees){
        int ticks = (int) (degrees * TICKS_PER_DEGREE);
        if (opMode.opModeIsActive()) {
            // Reset encoders
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            BL.setTargetPosition(-ticks);
            BR.setTargetPosition(ticks);
            FL.setTargetPosition(-ticks);
            FR.setTargetPosition(ticks);


            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            while ( opMode.opModeIsActive() && (
                    Math.abs(BL.getCurrentPosition() + ticks) > tolerance ||
                    Math.abs(BR.getCurrentPosition() - ticks) > tolerance ||
                    Math.abs(FL.getCurrentPosition() + ticks) > tolerance ||
                    Math.abs(FR.getCurrentPosition() - ticks) > tolerance)
            ) {
                opMode.telemetry.addData("BL pos", BL.getCurrentPosition());
                opMode.telemetry.addData("BR pos", BR.getCurrentPosition());
                opMode.telemetry.addData("FL pos", FL.getCurrentPosition());
                opMode.telemetry.addData("FR pos", FR.getCurrentPosition());
                opMode.telemetry.update();
            }
            opMode.sleep(100);
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }
    }

    public void strafe_left(double speed, double distance) {
        int ticks = (int) (distance * TICKS_PER_CM);  // Define ticks
        if (opMode.opModeIsActive()) {
            // Reset encoders
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FL.setTargetPosition(ticks);
            BL.setTargetPosition(-ticks);  // Strafing left is opposite for BL and FR
            FR.setTargetPosition(-ticks);  // Strafing left is opposite for BL and FR
            BR.setTargetPosition(ticks);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            while (opMode.opModeIsActive() && (
                    Math.abs(BL.getCurrentPosition() - (-ticks)) > tolerance ||
                    Math.abs(BR.getCurrentPosition() - ticks) > tolerance ||
                    Math.abs(FL.getCurrentPosition() - ticks) > tolerance ||
                    Math.abs(FR.getCurrentPosition() - (-ticks)) > tolerance
            )){
                opMode.telemetry.addData("BL pos", BL.getCurrentPosition());
                opMode.telemetry.addData("BR pos", BR.getCurrentPosition());
                opMode.telemetry.addData("FL pos", FL.getCurrentPosition());
                opMode.telemetry.addData("FR pos", FR.getCurrentPosition());
                opMode.telemetry.update();
            }
            opMode.sleep(100);
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }
    }

    // Move intake and outtake code to its own class
    public void intake() {
        lServo.setPower(1);
        rServo.setPower(1);
    }

    // Move intake and outtake code to its own class
    public void outtake() {
        lServo.setPower(-1);
        rServo.setPower(-1);

    }

    public void intakeStop() {
        lServo.setPower(0);
        rServo.setPower(0);
    }

    public void stickDrive() {
        // Implement logic for driving using gamepad sticks
    }
}

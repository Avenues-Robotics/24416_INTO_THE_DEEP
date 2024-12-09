package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive {
    LinearOpMode opMode;
    DcMotor FL;
    DcMotor BL;
    DcMotor FR;
    DcMotor BR;
    CRServo lServo;
    CRServo rServo;
    public DcMotor armSlideR;
    public DcMotor armSlideL;
    public static double TICKS_PER_CM = 17.5;
    public static double TICKS_PER_DEGREE = 12;

    public Drive(LinearOpMode opModeCalledFrom) {
        opMode = opModeCalledFrom;
        FL = opMode.hardwareMap.get(DcMotor.class, "FL");
        BL = opMode.hardwareMap.get(DcMotor.class, "BL");
        FR = opMode.hardwareMap.get(DcMotor.class, "FR");
        BR = opMode.hardwareMap.get(DcMotor.class, "BR");
        rServo = opMode.hardwareMap.get(CRServo.class, "rServo");
        lServo = opMode.hardwareMap.get(CRServo.class, "lServo");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);
    }

    // Method that drives the robot positive or negative a given distance and speed in cm
    public void drive(double speed, double distance) {
        int ticks = (int) (distance * TICKS_PER_CM);
        if (opMode.opModeIsActive()) {
            // Reset encoders
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FL.setTargetPosition(ticks);
            BL.setTargetPosition(ticks);
            FR.setTargetPosition(ticks);
            BR.setTargetPosition(ticks);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            while ( BL.getCurrentPosition() > ticks ||
                    BR.getCurrentPosition() > ticks ||
                    FL.getCurrentPosition() > ticks ||
                    FR.getCurrentPosition() > ticks){  //  BL.isBusy() || BR.isBusy() || FL.isBusy() || FR.isBusy() ||
                opMode.telemetry.addData("FL pos", FL.getCurrentPosition());
                opMode.telemetry.addData("BL pos", BL.getCurrentPosition());
                opMode.telemetry.addData("FR pos", FR.getCurrentPosition());
                opMode.telemetry.addData("BR pos", BR.getCurrentPosition());
                opMode.telemetry.update();
            }
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

            FL.setTargetPosition(-ticks);
            BL.setTargetPosition(-ticks);
            FR.setTargetPosition(ticks);
            BR.setTargetPosition(ticks);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FL.setPower(-speed);
            BL.setPower(-speed);
            FR.setPower(speed);
            BR.setPower(speed);
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){

            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }
    }

    public void Strafe_left(double speed, double distance) {
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
            BL.setPower(-speed);
            FR.setPower(-speed);
            BR.setPower(speed);

            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){

            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }
    }
    public void Strafe_right(double speed, double distance) {
        int ticks = (int) (distance * TICKS_PER_CM);  // Define ticks
        if (opMode.opModeIsActive()) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FL.setTargetPosition(-ticks);
            BL.setTargetPosition(ticks);  // Strafing left is opposite for BL and FR
            FR.setTargetPosition(ticks);
            BR.setTargetPosition(-ticks);  // Strafing left is opposite for BL and FR

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FL.setPower(-speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(-speed);

            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){

            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }
    }
    public void intake() {
        if (opMode.opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime();
            lServo.setPower(-0.5);
            rServo.setPower(-0.5);
            while (timer.milliseconds() < 2000) {

            }
            lServo.setPower(0);
            rServo.setPower(0);
        }
    }
    public void outtake() {
        if (opMode.opModeIsActive()){
            ElapsedTime timer = new ElapsedTime();
            lServo.setPower(0.5);
            rServo.setPower(0.5);
            while(timer.milliseconds() < 2000) {

            }
            lServo.setPower(0);
            rServo.setPower(0);
        }
    }

    public void stickDrive() {
        // Implement logic for driving using gamepad sticks
    }
}

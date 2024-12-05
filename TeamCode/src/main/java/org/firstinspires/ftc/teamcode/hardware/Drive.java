package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive {
    LinearOpMode opMode;
    DcMotor FL;
    DcMotor BL;
    DcMotor FR;
    DcMotor BR;
    public static double TICKS_PER_CM = 17.5;
    public static double TICKS_PER_DEGREE = 12;

    public Drive(LinearOpMode opModeCalledFrom) {
        opMode = opModeCalledFrom;
        FL = opMode.hardwareMap.get(DcMotor.class, "FL");
        BL = opMode.hardwareMap.get(DcMotor.class, "BL");
        FR = opMode.hardwareMap.get(DcMotor.class, "FR");
        BR = opMode.hardwareMap.get(DcMotor.class, "BR");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
    }

   // method that drives the robot positive or negitive. a given distance and speed in cm
    public void drive(double speed, double distance) {
        int ticks = (int) (distance * TICKS_PER_CM);
        if (opMode.opModeIsActive()) {
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
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }

    }
    public void rotate(double speed, double degrees){
        int ticks = (int) (degrees * TICKS_PER_DEGREE);
        if (opMode.opModeIsActive()) {
            // Resets the encoder
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

            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }

    }

    public void stickDrive(){
        // Use the gamepad sticks
    }

}

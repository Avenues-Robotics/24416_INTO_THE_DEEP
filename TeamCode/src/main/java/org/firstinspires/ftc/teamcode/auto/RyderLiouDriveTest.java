package org.firstinspires.ftc.teamcode.auto;
/******* This opMode is a full setup of rotation & drive functions. This is also a test for the linear slides*******/
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name="Ryder {:")
public class RyderLiouDriveTest extends LinearOpMode {

    // It is just defining the motors
    DcMotor FL;
    DcMotor BL;
    DcMotor FR;
    DcMotor BR;
    DcMotor AR;
    public static double TICKS_PER_CM = 17.5;
    public static double TICKS_PER_DEGREE = 12;
    double targetPosition = 244;
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    double Kf = 5;
    double Kp = 0.5;
    double Ki = 0.1;
    double Kd = 0.5;
    double finalPower;
    double error;
    double deptritive;

    double encoderPosition = AR.getCurrentPosition();
    double lastError = 0;

    @Override
    public void runOpMode(){


        // Create two DcMotor OBJECTS connected to the motors on
        // the robot named motorLeft and motorRight
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // saying that the left motor is reversed to it will swap the direction on code so you don't need to insert information manually
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        // Set the mode to Run using encoder
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // It makes it so that when you start the code, it will start.
        waitForStart();
        while(Math.abs(encoderPosition - targetPosition) > 5){
            error = targetPosition - encoderPosition;
            deptritive = (error - lastError)/timer.seconds();
            integralSum = integralSum + (error * timer.seconds());
            finalPower = (integralSum*Ki)+(deptritive*Kd)+(error*Kp)+Kf;
            AR.setPower(finalPower);
            lastError = error;

            timer.reset();
        }

        // Play around with the PARAMETERS inside the parentheses
        // Goes forward

        // Add a comment to explain what the two arguments in the addData() method do

    }

    public void drive(double speed, double distance){
        int ticks = (int) (distance * TICKS_PER_CM);
        if (opModeIsActive()) {

            // Resets the encoder
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // allows you to set the motor to a certain position
            FL.setTargetPosition(ticks);
            BL.setTargetPosition(ticks);
            FR.setTargetPosition(ticks);
            BR.setTargetPosition(ticks);

            // It runs to the certain position mode
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // you set the speed of the motor
            FL.setPower(speed);
            BL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);

            // If you are controling the robot and the motors are "busy" then
            while (opModeIsActive() &&
                    (FL.isBusy() && BL.isBusy() && BR.isBusy() && FR.isBusy())) {

                // Gets the data about what the motor is doing
                telemetry.addData("Running to", "Left: " + ticks + " | Right: " + ticks);
                telemetry.addData("Current pos", "Left: " + BL.getCurrentPosition() + " | Right: " + BL.getCurrentPosition());
                telemetry.update();
            }

            // gets current position of robot/motors
            telemetry.addData("Running to", "Left: " + ticks + " | Right: " + ticks);
            telemetry.addData("Current pos", "Left: " + FL.getCurrentPosition() + " | Right: " + FL.getCurrentPosition());
            telemetry.update();

            // stops the motor
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);

        }
    }
    public void rotate(double speed, double degrees){
        int ticks = (int) (degrees * TICKS_PER_DEGREE);
        if (opModeIsActive()) {
            // Resets the encoder
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // allows you to set the motor to a certain position
            FL.setTargetPosition(-ticks);
            BL.setTargetPosition(-ticks);
            FR.setTargetPosition(ticks);
            BR.setTargetPosition(ticks);

            // It runs to the certain position mode
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // you set the speed of the motor
            FL.setPower(-speed);
            BL.setPower(-speed);
            FR.setPower(speed);
            BR.setPower(speed);

            // If you are controling the robot and the motors are "busy" then
            while (opModeIsActive() &&
                    (FL.isBusy() && BL.isBusy())) {

                // Gets the data about what the motor is doing
                telemetry.addData("Running to", "Left: " + ticks + " | Right: " + ticks);
                telemetry.addData("Current pos", "Left: " + FL.getCurrentPosition() + " | Right: " + FL.getCurrentPosition());
                telemetry.update();
            }

            // gets current position of robot/motors
            telemetry.addData("Running to", "Left: " + ticks + " | Right: " + ticks);
            telemetry.addData("Current pos", "Left: " + BL.getCurrentPosition() + " | Right: " + BL.getCurrentPosition());
            telemetry.update();

            // stops the motor
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }

    }

}

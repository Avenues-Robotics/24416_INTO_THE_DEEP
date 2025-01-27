package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.PIDF;

@Config
public class Slides {
    LinearOpMode opMode;
    public DcMotor armSlideR;
    public DcMotor armSlideL;

    public PIDF leftSlidePIDF;
    public PIDF rightSlidePIDF;

    public double targetPosition = 0;
    private static double KpL = 0.015;
    private static double KpR = 0.015;
    private static double KiR = 0;
    private static double KiL = 0;
    private static double KdR = 0;
    private static double KdL = 0;
    private static double KfR = 0.175;
    private static double KfL = 0.175;
    private static double toleranceL = 10;
    private static double toleranceR = 10;
    public static double Kstick;

    public static int closeIntakePos = 850; // CHECKED
    public static int CloseIntakeAutoPos = 1200;
    public static int lowOuttakePos = 1200; // CHECKED
    public static int highOuttakePos = 2750; // CHECKED
    public static int slidesRetractedPos = 0;
    public static int maxHorizontalPos = 1400; // CHECKED

    public Slides(LinearOpMode opModeCalledFrom){
        opMode = opModeCalledFrom;
        armSlideR = opMode.hardwareMap.get(DcMotor.class, "ArmSlideR");
        armSlideL = opMode.hardwareMap.get(DcMotor.class, "ArmSlideL");
        armSlideR.setDirection(DcMotor.Direction.REVERSE);
        armSlideL.setDirection(DcMotor.Direction.FORWARD);
        leftSlidePIDF = new PIDF(KpL, KiL, KdL, KfL, toleranceL);
        rightSlidePIDF = new PIDF(KpR, KiR, KdR, KfR, toleranceR);
        armSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setState(String state, Rotate rotate){

        if(state.equals("SLIDES RETRACTED")){
            targetPosition = slidesRetractedPos;
        }
        else if(state.equals("CLOSE INTAKE")){
            targetPosition = closeIntakePos;
        }
        else if(state.equals("CLOSE AUTO INTAKE")){
            targetPosition = CloseIntakeAutoPos;
        }
        else if(state.equals("LOW OUTTAKE")){
            targetPosition = lowOuttakePos;
        }
        else if(state.equals("HIGH OUTTAKE")) {
            targetPosition = highOuttakePos;
        }


        // SET POWER USING TARGET POSITION
        double currentValueR = armSlideR.getCurrentPosition();
        double currentValueL = armSlideL.getCurrentPosition();
        double finalPowerR = rightSlidePIDF.update(targetPosition, currentValueR);
        double finalPowerL = leftSlidePIDF.update(targetPosition, currentValueR);

        // DEFAULT TO 0 POWER IF OUTTAKE AND RETRACTED
        if(state.equals("SLIDES RETRACTED") && getPosition() < 200 && rotate.getPosition() >= -50){
            finalPowerR = 0;
            finalPowerL = 0;
        }
        if(state.equals("MANUAL")){
            double stick = opMode.gamepad2.left_stick_x;
            if(Math.abs(stick) > 0.1){
                if(rotate.getPosition() < -120 && getPosition() <= maxHorizontalPos){ // INTAKING
                    Kstick = 0.4;
                    finalPowerL = stick * Kstick;
                    finalPowerR = stick * Kstick;

                }
                else if(rotate.getPosition() > -25 && getPosition() <= highOuttakePos){ //OUTTAKING
                    Kstick = 0.5;
                    finalPowerL = stick * Kstick;
                    finalPowerR = stick * Kstick;
                }
                else{
                    Kstick = 0.5;
                    finalPowerL = stick * Kstick;
                    finalPowerR = stick * Kstick;
                }
            }
        }
        else if(state.equals("HOLD")){
            finalPowerR = rightSlidePIDF.update(targetPosition, currentValueR);
            finalPowerL = leftSlidePIDF.update(targetPosition, currentValueR);
        }

        armSlideR.setPower(finalPowerR);
        armSlideL.setPower(finalPowerL);
    }

    public int getPosition(){
        return (armSlideR.getCurrentPosition() + armSlideR.getCurrentPosition()) / 2;
    }

    public void setTargetPosition(int position){
        targetPosition = position;
    }


}

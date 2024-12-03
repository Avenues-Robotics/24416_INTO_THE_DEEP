package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.PIDF;

public class Slides {
    LinearOpMode opMode;
    public DcMotor armSlideR;
    public DcMotor armSlideL;

    public PIDF leftSlidePIDF;
    public PIDF rightSlidePIDF;

    public static double targetPosition = 0;
    public static double KpL = 0.015;
    public static double KpR = 0.015;
    public static double KiR = 0;
    public static double KiL = 0;
    public static double KdR = 0;
    public static double KdL = 0;
    public static double KfR = 0.175;
    public static double KfL = 0.175;
    public static double toleranceL = 10;
    public static double toleranceR = 10;

    public Slides(LinearOpMode opModeCalledFrom){
        opMode = opModeCalledFrom;
        armSlideR = opModeCalledFrom.hardwareMap.get(DcMotor.class, "ArmSlideR");
        armSlideL = opModeCalledFrom.hardwareMap.get(DcMotor.class, "ArmSlideL");
        armSlideR.setDirection(DcMotor.Direction.REVERSE);
        armSlideL.setDirection(DcMotor.Direction.FORWARD);
        leftSlidePIDF = new PIDF(KpL, KiL, KdL, KfL, toleranceL);
        rightSlidePIDF = new PIDF(KpR, KiR, KdR, KfR, toleranceR);
        armSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPostion(String state){
        int targetPosition;
        if(state.equals("RETRACTED")){
            targetPosition = 0;
        }
        else if(state.equals("CLOSE INTAKE")){
            targetPosition = 500; // FIX THIS VALUE
        }
        else if(state.equals("FAR INTAKE")){
            targetPosition = 2500; // FIX THIS VALUE
        }
        else if(state.equals("OUTTAKE")){
            targetPosition = 3000; // FIX THIS VALUE
        }
        else{
            targetPosition = 0;
        }

        double currentValueR = armSlideR.getCurrentPosition();
        double currentValueL = armSlideL.getCurrentPosition();
        double finalPowerR = rightSlidePIDF.update(targetPosition, currentValueR);
        double finalPowerL = leftSlidePIDF.update(targetPosition, currentValueL);
        armSlideR.setPower(finalPowerR);
        armSlideL.setPower(finalPowerL);
    }

    public int getPosition(){
        return (armSlideL.getCurrentPosition() + armSlideR.getCurrentPosition()) / 2;
    }


}

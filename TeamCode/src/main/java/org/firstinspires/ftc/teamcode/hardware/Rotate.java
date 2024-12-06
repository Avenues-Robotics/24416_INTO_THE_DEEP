package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.PIDF;

public class Rotate {

    LinearOpMode opMode;
    public DcMotor armRotateR;
    public DcMotor armRotateL;

    PIDF intakePIDF;
    PIDF outakePIDF;

    public static double intake_Kp = 0.0105;
    public static double intake_Ki = 0;
    public static double intake_Kd = 0.0015;
    public static double intake_Kf = 0;

    public static double outtake_Kp = 0.0025;
    public static double outtake_Ki = 0;
    public static double outtake_Kd = 0.000011;
    public static double outtake_Kf = 0;

    public static double tolerance = 0;

    public static int outtakepos = 0;

    public static int intakepos = -238;

    public int targetPosition;

    double power;

    public Rotate(LinearOpMode opModeCalledFrom){
        opMode = opModeCalledFrom;
        armRotateR = opMode.hardwareMap.get(DcMotor.class, "ArmRotateR");
        armRotateL = opMode.hardwareMap.get(DcMotor.class, "ArmRotateL");
        armRotateR.setDirection(DcMotor.Direction.REVERSE);
        armRotateL.setDirection(DcMotor.Direction.FORWARD);
        armRotateR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotateL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotateR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotateL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePIDF = new PIDF(intake_Kp, intake_Ki, intake_Kd, intake_Kf, tolerance);
        outakePIDF = new PIDF(outtake_Kp, outtake_Ki, outtake_Kd, outtake_Kf, tolerance);
    }

    public void setState(String state){

        int currentValue = (armRotateR.getCurrentPosition() + armRotateL.getCurrentPosition()) / 2;

        if (state.equals("INTAKE")) {
            targetPosition = intakepos;
            if(currentValue <= targetPosition + 20){
                power = 0;
            } else{
                int x = currentValue;
                intakePIDF.setKf(-(0.5142 * Math.sin(0.01389*x + 2.037) - 0.05517));
                power = intakePIDF.update(targetPosition, currentValue);

            }

        } else if (state.equals("OUTTAKE")) {
            targetPosition = outtakepos;
            if(currentValue >= targetPosition - 20){
                power = 0;
            } else{
                int x = currentValue;
                outakePIDF.setKf(-(0.5142 * Math.sin(0.01389*x + 2.037) - 0.05517));
                power = outakePIDF.update(targetPosition, currentValue);
            }
        } else{
            power = 0;
        }

        armRotateR.setPower(power);
        armRotateL.setPower(power);
    }

    public int getPosition(){
        return (armRotateL.getCurrentPosition() + armRotateR.getCurrentPosition()) / 2;
    }

}

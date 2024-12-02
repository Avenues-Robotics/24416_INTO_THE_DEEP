package org.firstinspires.ftc.teamcode.Control_Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.utilities.PIDF;

import java.util.Objects;

@Config
@Autonomous(name="Ryder rotation test")
public class ArmRotation_prediction extends LinearOpMode {
    Slides slides;
    DcMotor armRotateR;
    DcMotor armRotateL;

    PIDF intakePIDF;
    PIDF outakePIDF;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    //adjust the Kp Ki Kf Kd to diffrent things based opon if it needs to go forward or back

    String state;
    public static double targetPosition = 0;

    public static double intake_Kp = 0.0105;
    public static double intake_Ki = 0;
    public static double intake_Kd = 0.0015;
    public static double intake_Kf = 0;

    public static double outtake_Kp = 0.0025;
    public static double outtake_Ki = 0;
    public static double outtake_Kd = 0.000011;
    public static double outtake_Kf = 0;

    public static double tolerance = 0;

    public static int outtakepos = 225;

    public static int intakepos = -225;


    int slidesTarget = 0;


    public void runOpMode() {
        // Initialize slides
        slides = new Slides(this);

        // Initialization arm motors
        armRotateR = hardwareMap.get(DcMotor.class, "ArmRotateR");
        armRotateL = hardwareMap.get(DcMotor.class, "ArmRotateL");
        armRotateR.setDirection(DcMotor.Direction.REVERSE);
        armRotateL.setDirection(DcMotor.Direction.FORWARD);
        armRotateR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotateL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotateR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotateL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePIDF = new PIDF(intake_Kp, intake_Ki, intake_Kd, intake_Kf, tolerance);

        outakePIDF = new PIDF(outtake_Kp, outtake_Ki, outtake_Kd, outtake_Kf, tolerance);

        while (opModeInInit()) {
            dashboardTelemetry.addData("motor rotate position", armRotateR.getCurrentPosition());
            dashboardTelemetry.addData("motor rotate position", armRotateL.getCurrentPosition());
            dashboardTelemetry.update();
        }

        waitForStart();

        double power;
        // Control Loop

        while (opModeIsActive()) {

            // Slides
            double currentValueR = slides.armSlideR.getCurrentPosition();
            double currentValueL = slides.armSlideL.getCurrentPosition();
            double finalPowerR = slides.rightSlidePIDF.update(slidesTarget, currentValueR);
            double finalPowerL = slides.leftSlidePIDF.update(slidesTarget, currentValueL);
            slides.armSlideR.setPower(finalPowerR);
            slides.armSlideL.setPower(finalPowerL);

            // Rotation
            int currentValue = (armRotateR.getCurrentPosition() + armRotateL.getCurrentPosition()) / 2;

//            //repeat for all possible values of the 9 positions test to find the kps
//            if (pos0 < currentValue && currentValue < pos1 && state == "OUTAKE") {
//                //set a value that holds it in place to a degree/slows it down
//                outtake_Kf = 0;
//            }
//            if (pos0 < currentValue && currentValue < pos1 && state == "INTAKE") {
//                //set a value that holds it in place to a degree/slows it down
//                intake_Kf = 0;
//            }

            // eventually use gamepad to set state
            // eventually use gamepad to set state
            // eventually use gamepad to set state
            // eventually use gamepad to set state
            // eventually use gamepad to set state
            state = "INTAKE";

            if (state.equals("INTAKE")) {
                targetPosition=intakepos;
                if(currentValue <= targetPosition){
                    power = 0;
                } else{
                    int x = currentValue;
                    intakePIDF.setKf(0.5142 * Math.sin(0.01389*x + 2.037) - 0.05517);
                    power = intakePIDF.update(targetPosition, currentValue);
                }

            } else if (state.equals("OUTTAKE")) {
                targetPosition = outtakepos;
                if (currentValue >= targetPosition - 5) {
                    power = 0;
                } else {
                    int x = currentValue;
                    outakePIDF.setKf(0.5142 * Math.sin(0.01389 * x + 2.037) - 0.05517);
                    power = outakePIDF.update(targetPosition, currentValue);
                }
            }else{
                power = 0;
//                targetPosition = 0;//put the tick value of the intake position;
//                power = intakePIDF.update(targetPosition, currentValue);
            }

            armRotateR.setPower(power);
            armRotateL.setPower(power);

            dashboardTelemetry.addData("motors pos", currentValue);
            dashboardTelemetry.addData("motor left pow", armRotateL.getPower());
            dashboardTelemetry.addData("motor right pow", armRotateR.getPower());
            dashboardTelemetry.update();
        }
    }
}

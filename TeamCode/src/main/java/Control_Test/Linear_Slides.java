package Control_Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.PIDF;

@Config
@Autonomous(name="Ryder Linear slide test")
public class Linear_Slides extends LinearOpMode {
    DcMotor armSlideR;
    DcMotor armSlideL;

    PIDF leftSlidePIDF;
    PIDF rightSlidePIDF;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static double targetPosition = 0;
    public static double KpL = 0;
    public static double KpR = 0;
    public static double KiR = 0;
    public static double KiL = 0;
    public static double KdR = 0;
    public static double KdL = 0;
    public static double KfR = 0.175;
    public static double KfL = 0.175;
    public static double toleranceL = 5;
    public static double toleranceR = 5;



    public void runOpMode() {
        // Initialization
        armSlideR = hardwareMap.get(DcMotor.class, "ArmSlideR");
        armSlideL = hardwareMap.get(DcMotor.class, "ArmSlideL");
        armSlideR.setDirection(DcMotor.Direction.REVERSE);
        armSlideL.setDirection(DcMotor.Direction.FORWARD);
        leftSlidePIDF = new PIDF(KpL, KiL, KdL, KfL, toleranceL);
        rightSlidePIDF = new PIDF(KpR, KiR, KdR, KfR, toleranceR);
        armSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dashboardTelemetry.addData("motor left position", armSlideL.getCurrentPosition());
        dashboardTelemetry.addData("motor right position", armSlideR.getCurrentPosition());
        dashboardTelemetry.update();
        waitForStart();

        // Control Loop
        while (opModeIsActive()) {
            double currentValueR = armSlideR.getCurrentPosition();
            double currentValueL = armSlideL.getCurrentPosition();
            double finalPowerR = rightSlidePIDF.update(targetPosition, currentValueR);
            double finalPowerL = leftSlidePIDF.update(targetPosition, currentValueL);
            armSlideR.setPower(finalPowerR);
            armSlideL.setPower(finalPowerL);
            dashboardTelemetry.addData("motor left position", currentValueL);
            dashboardTelemetry.addData("motor right position", currentValueR);
            dashboardTelemetry.addData("motor right power", finalPowerR);
            dashboardTelemetry.addData("motor left power", finalPowerL);
            dashboardTelemetry.update();
        }

    }

    public static class ArmRotation {
    }
}
package org.firstinspires.ftc.teamcode.Control_Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;

@Config
@Autonomous(name="Just Telemetry", group="Tests")
public class JustTelemetry extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Slides slides;
    Drive drive;
    String slidesState;
    String rotateState ;
    Rotate rotate;

    public void runOpMode() {

        slides = new Slides(this);
        rotate = new Rotate(this);
        drive = new Drive(this);

        drive.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeInInit()) {
            telemetry.addData("Slide position", slides.getPosition());
            telemetry.addData("Rotate position", rotate.getPosition());
            telemetry.addData("BL pos", drive.BL.getCurrentPosition());
            telemetry.addData("BR pos", drive.BR.getCurrentPosition());
            telemetry.addData("FL pos", drive.FL.getCurrentPosition());
            telemetry.addData("FR pos", drive.FR.getCurrentPosition());
            telemetry.update();

            dashboardTelemetry.addData("Slide position", slides.getPosition());
            dashboardTelemetry.addData("Rotate position", rotate.getPosition());
            dashboardTelemetry.update();
        }
    }


}

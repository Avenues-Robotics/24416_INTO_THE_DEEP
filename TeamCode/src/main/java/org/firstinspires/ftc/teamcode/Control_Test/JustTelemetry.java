package org.firstinspires.ftc.teamcode.Control_Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;

@Config
@Autonomous(name="Just Telemetry", group="Tests")
public class JustTelemetry extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Slides slides;
    String slidesState;
    String rotateState ;
    Rotate rotate;

    public void runOpMode() {

        slides = new Slides(this);
        rotate = new Rotate(this);

        while (opModeInInit()) {
            telemetry.addData("Slide position", slides.getPosition());
            telemetry.addData("Rotate position", rotate.getPosition());
            telemetry.update();

            dashboardTelemetry.addData("Slide position", slides.getPosition());
            dashboardTelemetry.addData("Rotate position", rotate.getPosition());
            dashboardTelemetry.update();
        }
    }


}

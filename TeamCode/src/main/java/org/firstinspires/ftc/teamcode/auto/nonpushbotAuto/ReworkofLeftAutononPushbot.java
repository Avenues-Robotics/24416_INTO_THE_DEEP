package org.firstinspires.ftc.teamcode.auto.nonpushbotAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.StartServo;

@Config
@Autonomous(name="Left Sample 1234")
public class ReworkofLeftAutononPushbot extends LinearOpMode {
    Drive drive;
    Slides slides;
    Rotate rotate;
    StartServo startServo;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotor FR;
    DcMotor FL;
    DcMotor BL;
    DcMotor BR;
    ElapsedTime timer = new ElapsedTime();

    double distance;

    public static double ticks_per_degree = 10.7;
    public static int strafe_1 = -15;
    public static int rotate_1 = -45;
    public static int drive_1 = 59;
    public static int rotate_2 = 45;
    public static int rotate_3 = -45;
    public static int drive_2 = -18;
    public static int strafe_2 = 15;
    public static int Drive_3 = -10;
    public static int drive_4 = 20;
    public static int time_ms = 2000;


    @Override
    public void runOpMode() {
        CRServo rServo = hardwareMap.get(CRServo.class, "rServo");
        CRServo lServo = hardwareMap.get(CRServo.class, "lServo");
        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);
        drive = new Drive(this);
        Drive.TICKS_PER_DEGREE = ticks_per_degree;
        slides = new Slides(this);
        rotate = new Rotate(this);
        startServo = new StartServo(this);
        startServo.start();

        waitForStart();
        drive.drive(0.6, drive_1);
        rotate.armRotateL.setPower(0.5);
        rotate.armRotateR.setPower(0.5);
        sleep(300);
        rotate.armRotateL.setPower(0);
        rotate.armRotateR.setPower(0);
        startServo.open();
        drive.rotate(0.5, rotate_1);
        drive.drive(0.6, drive_2);
        while (opModeIsActive() && slides.getPosition() <= slides.highOuttakePos){
            slides.setState("HIGH OUTTAKE", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            drive.outtake();
            slides.setState("HIGH OUTTAKE", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.intakeStop();
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+25){
            slides.setState("SLIDES RETRACTED", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            slides.setState("SLIDES RETRACTED", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.rotate(0.6, rotate_2);
        drive.strafe_left(0.6, strafe_1);
        drive.drive(0.6, Drive_3);
        while (opModeIsActive() && rotate.getPosition() >= rotate.intakepos) {
            rotate.setState("INTAKE");
        }
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            rotate.setState("INTAKE");
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.intake();
        while (opModeIsActive() && slides.getPosition() >= slides.closeIntakePos){
            slides.setState("CLOSE INTAKE", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            slides.setState("CLOSE INTAKE", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.drive(0.3, drive_4);
        drive.intakeStop();
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+25){
            slides.setState("SLIDES RETRACTED", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            slides.setState("SLIDES RETRACTED", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        while (opModeIsActive() && rotate.getPosition() >= rotate.outtakepos) {
            rotate.setState("OUTTAKE");
            slides.setState("SLIDES RETRACTED", rotate);

        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            rotate.setState("OUTTAKE");
            slides.setState("SLIDES RETRACTED", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.strafe_left(0.6, strafe_2);
        drive.rotate(0.6, rotate_3);
        while (opModeIsActive() && slides.getPosition() <= slides.highOuttakePos){
            slides.setState("HIGH OUTTAKE", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            drive.outtake();
            slides.setState("HIGH OUTTAKE", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.intakeStop();
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+25){
            slides.setState("SLIDES RETRACTED", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            slides.setState("SLIDES RETRACTED", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
       }
      }

package org.firstinspires.ftc.teamcode.auto.nonpushbotAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.StartServo;

@Config
@Autonomous(name="Left Auto Specimen and Sample 1234")
@Disabled
public class LeftfirstnonpushbotAuto extends LinearOpMode {
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
    private DistanceSensor sensorDistance;
    private DistanceSensor sensorDistance2;

    double distance;
    public static int strafe_1 = 20;
    public static int rotate_1 = 90;
    public static int drive_1 = -60;
    public static int rotate_2 = -90;
    public static int rotate_3 = -90;
    public static int drive_2 = 40;
    public static int strafe_2 = -15;
    public static int Drive_3 = 99;
    public static int drive_4 = 20;

    @Override
    public void runOpMode() {
        CRServo rServo = hardwareMap.get(CRServo.class, "rServo");
        CRServo lServo = hardwareMap.get(CRServo.class, "lServo");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance1");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);
        drive = new Drive(this);
        slides = new Slides(this);
        rotate = new Rotate(this);
        startServo = new StartServo(this);
        startServo.start();

        waitForStart();
//        rotate.armRotateL.setPower(0.5);
//        rotate.armRotateR.setPower(0.5);
//        sleep(300);
//        rotate.armRotateL.setPower(0);
//        rotate.armRotateR.setPower(0);

        distance = (sensorDistance.getDistance(DistanceUnit.CM) + sensorDistance2.getDistance(DistanceUnit.CM)) / 2;
        telemetry.addData("distance", distance);
        telemetry.update();

            //        } FOR TESTING
        startServo.open();
        drive.strafe_left(0.6, strafe_1);
        drive.rotate(0.6, rotate_1);
        drive.strafe_left(0.6, strafe_2);
        drive.drive(0.6, drive_1);
        while(opModeIsActive() && slides.getPosition() < Slides.closeIntakePos) {
            slides.setState("CLOSE INTAKE", rotate);
        }
        while(opModeIsActive() && slides.getPosition() >= Slides.slidesRetractedPos + 5) {
            slides.setState("SLIDES RETRACTED", rotate);
        }
        drive.drive(0.6, drive_2);
        drive.rotate(0.6, rotate_2);
        drive.drive(0.8, Drive_3);
//        distance = (sensorDistance.getDistance(DistanceUnit.CM) + sensorDistance2.getDistance(DistanceUnit.CM))/2;
//      SUPER COOL DISTANCE SENSOR CODE
//        telemetry.addData("distance", distance);
//        telemetry.update();
//        while(distance > opDrive_3){
//            FR.setPower(0.5);
//            FL.setPower(0.5);
//            BR.setPower(0.5);
//            BL.setPower(0.5);
//            distance = (sensorDistance.getDistance(DistanceUnit.CM) + sensorDistance2.getDistance(DistanceUnit.CM))/2;
//            telemetry.addData("distance", distance);
//            telemetry.update();

        drive.rotate(0.6, rotate_3);   startServo.open();
        drive.strafe_left(0.6, strafe_1);
        drive.rotate(0.6, rotate_1);
        drive.strafe_left(0.6, strafe_2);
        drive.drive(0.6, drive_1);
        while(opModeIsActive() && slides.getPosition() < Slides.closeIntakePos) {
            slides.setState("CLOSE INTAKE", rotate);
        }
        while(opModeIsActive() && slides.getPosition() >= Slides.slidesRetractedPos + 5) {
            slides.setState("SLIDES RETRACTED", rotate);
        }
        drive.drive(0.6, drive_2);
        drive.rotate(0.6, rotate_2);
        drive.drive(0.8, Drive_3);
//        distance = (sensorDistance.getDistance(DistanceUnit.CM) + sensorDistance2.getDistance(DistanceUnit.CM))/2;
//      SUPER COOL DISTANCE SENSOR CODE
//        telemetry.addData("distance", distance);
//        telemetry.update();
//        while(distance > opDrive_3){
//            FR.setPower(0.5);
//            FL.setPower(0.5);
//            BR.setPower(0.5);
//            BL.setPower(0.5);
//            distance = (sensorDistance.getDistance(DistanceUnit.CM) + sensorDistance2.getDistance(DistanceUnit.CM))/2;
//            telemetry.addData("distance", distance);
//            telemetry.update();
//        }
        drive.rotate(0.6, rotate_3);
//       Set intake position to -225?
//        while(opModeIsActive() && rotate.getPosition() >= Rotate.intakepos) {
//            rotate.setState("INTAKE");
//        }
//        // USe sleep?
            while (opModeIsActive() && rotate.getPosition() > (rotate.intakepos + 15)) {
                rotate.setState("INTAKE");
            }
            sleep(500);

            while (opModeIsActive() && slides.getPosition() < Slides.CloseIntakeAutoPos) {
                // set to Manual?
                slides.setState("CLOSE AUTO INTAKE", rotate);
            }
            drive.intake();
            drive.drive(0.6, drive_4);
            sleep(2500);
            drive.intakeStop();
            while (opModeIsActive() && slides.getPosition() > (Slides.slidesRetractedPos)) {
                slides.setState("SLIDES RETRACTED", rotate);
            }
            while (opModeIsActive() && rotate.getPosition() < Rotate.outtakepos) {
                rotate.setState("OUTTAKE");
            }

    }
}


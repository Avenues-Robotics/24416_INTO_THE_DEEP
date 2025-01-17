package org.firstinspires.ftc.teamcode.auto.nonpushbotAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name="Left Auto 1234")
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

    public static double ticks_per_degree = 10.7;
    public static int strafe_1 = 20;
    public static int rotate_1 = 90;
    public static int drive_1 = -60;
    public static int rotate_2 = -90;
    public static int rotate_3 = -90;
    public static int drive_2 = 40;
    public static int strafe_2 = -15;
    public static int opDrive_3 = 30;
    public static int drive_4 = 10;
    double distance = (sensorDistance.getDistance(DistanceUnit.CM) + sensorDistance2.getDistance(DistanceUnit.CM))/2;
    @Override
    public void runOpMode() {
        CRServo rServo = hardwareMap.get(CRServo.class, "rServo");
        CRServo lServo = hardwareMap.get(CRServo.class, "lServo");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance1");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);
        drive = new Drive(this);
        Drive.TICKS_PER_DEGREE = ticks_per_degree;
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
        startServo.open();
        drive.strafe_left(1, strafe_1);
        drive.rotate(1, rotate_1);
        drive.strafe_left(1, strafe_2);
        drive.drive(1, drive_1);
        while(opModeIsActive() && slides.getPosition() < Slides.closeIntakePos) {
            slides.setState("CLOSE INTAKE", rotate);
        }
        lServo.setPower(1);
        rServo.setPower(1);
        while(opModeIsActive() && slides.getPosition() >= Slides.slidesRetractedPos + 5) {
            slides.setState("SLIDES RETRACTED", rotate);
        }
        lServo.setPower(0);
        rServo.setPower(0);
        drive.drive(0.5, drive_2);
        drive.rotate(0.5, rotate_2);
        while(distance > opDrive_3){
            FR.setPower(0.5);
            FL.setPower(0.5);
            BR.setPower(0.5);
            BL.setPower(0.5);
        }
        drive.rotate(0.5, rotate_3);
//       Set intake position to -225?
//        while(opModeIsActive() && rotate.getPosition() >= Rotate.intakepos) {
//            rotate.setState("INTAKE");
//        }
        // USe sleep?
        rotate.setState("INTAKE");
        sleep(500);
        while(opModeIsActive() && slides.getPosition() < Slides.CloseIntakeAutoPos)  {
//            slides.setState("CLOSE AUTO INTAKE", rotate);
            // set to Manual?
            slides.setState("CLOSE INTAKE", rotate);
        }
        drive.drive(1, drive_4);
        drive.intake();






    }
}

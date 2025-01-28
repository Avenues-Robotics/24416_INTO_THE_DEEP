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
@Autonomous(name="Left 2 Sample 1234")
public class ReworkofLeftAutononPushbot extends LinearOpMode {
    Drive drive;
    Slides slides;
    Rotate rotate;
    StartServo startServo;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    ElapsedTime timer = new ElapsedTime();

    double distance;

    public static double ticks_per_degree = 10.7;
    public static int strafe_1 = -10;
    public static int rotate_1 = -45;
    public static int drive_1 = 54;
    public static int rotate_2 = 45;
    public static int rotate_3 = -45;
    public static int drive_2 = -15;
    public static int strafe_2 = 20;
    public static int Drive_3 = -10;
    public static int drive_4 = 20;
    public static int drive_5 = -10;
    public static int strafe_3 = -40;
    public static int rotate_4 = 45;
    public static int drive_6 = 10;
    public static int drive_7 = 20;
    public static int rotate_5 = -45;
    public static int drive_8 = 10;
    public static int drive_start = 5;
    public static int time_ms = 2000;



    @Override
    public void runOpMode() {

        // INITIALIZE
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

        // RUN

        dashboardTelemetry.addLine("Should be 0");
        dashboardTelemetry.addData("rotate", rotate.getPosition());
        dashboardTelemetry.addData("rotate L", rotate.getPositionL());
        dashboardTelemetry.addData("rotate R", rotate.getPositionR());
        dashboardTelemetry.update();

        // 1. DRIVE FORWARD
        drive.drive(0.3, drive_start);

        // 2. ROTATE ARM TO 0 POSITION
        rotate.armRotateL.setPower(0.7);
        rotate.armRotateR.setPower(0.7);
        sleep(300);
        rotate.armRotateL.setPower(0);
        rotate.armRotateR.setPower(0);

        // THIS SEEMED TO FIX THE ROTATION ISSUE. EACH TIME BEFORE YOU
        // ROTATE THE SLIDES TO THE INTAKE POSITION, RUN THESE FOUR LINES.
        // TO MAKE IT EASIER, MAKE A METHOD IN YOUR Rotate CLASS TO
        // RUN THESE FOUR LINES FOR YOU.  SOMETHING LIKE rezeroRotation()

        // RESET MOTOR MODES
        rotate.resetRotation();

        // DEBUG: CHECK ROTATION POSITONS
        dashboardTelemetry.addLine("Should be 0");
        dashboardTelemetry.addData("rotate", rotate.getPosition());
        dashboardTelemetry.addData("rotate L", rotate.getPositionL());
        dashboardTelemetry.addData("rotate R", rotate.getPositionR());
        dashboardTelemetry.update();

        startServo.open();
        sleep(500);

        // 3. DRIVE TO LINE UP SAMPLE 1 DELIVERY
        drive.drive(0.7, drive_1);
        drive.rotate(0.5, rotate_1);
        drive.drive(0.7, drive_2);

        // 4. DELIVER SAMPLE 1
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
        slides.resetSLides();

//        // Not sure what this does
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < time_ms){
//            slides.setState("SLIDES RETRACTED", rotate);
//            dashboardTelemetry.addData("timer", timer);
//            dashboardTelemetry.update();
//        }

        // 5. DRIVE TO SAMPLE 2 INTAKE
        drive.rotate(0.7, rotate_2);
        drive.strafe_left(0.7, strafe_1);
        drive.drive(0.7, Drive_3);

        // 6. ROTATE SLIDES TO INTAKE POSITION

        //  ** PROBLEM **
        // For some reason rotateL and rotateR are returning about -113 not -235
        timer.reset();

        // RESET MOTOR MODES
        rotate.resetRotation();

        while (opModeIsActive() && rotate.getPosition() >= rotate.intakepos + 25) {
            slides.setState("SLIDES RETRACTED", rotate);
            rotate.setState("INTAKE");
            dashboardTelemetry.addLine("IN LOOP Should be going to -210");
            dashboardTelemetry.addData("rotate", rotate.getPosition());
            dashboardTelemetry.addData("rotate L", rotate.getPositionL());
            dashboardTelemetry.addData("rotate R", rotate.getPositionR());
            dashboardTelemetry.addData("rotate power", rotate.getPower());
            dashboardTelemetry.addData("timer", timer.seconds());
            dashboardTelemetry.update();
        }

        // Not sure what this does
//        while(opModeIsActive() && timer.milliseconds() < time_ms){
//            rotate.setState("INTAKE");
//            dashboardTelemetry.addData("timer", timer);
//            dashboardTelemetry.update();
//        }

        // 7. INTAKE SAMPLE 2
        drive.intake();
        while (opModeIsActive() && slides.getPosition() <= slides.closeIntakePos){
            slides.setState("CLOSE INTAKE", rotate);
        }
//        // Not sure what this does
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < time_ms){
//            slides.setState("CLOSE INTAKE", rotate);
//            dashboardTelemetry.addData("timer", timer);
//            dashboardTelemetry.update();
//        }

        // SUGGESTION: Change this to slowly extend slides.
        drive.drive(0.15, drive_4);
        drive.intakeStop();
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+25){
            slides.setState("SLIDES RETRACTED", rotate);
        }
        slides.resetSLides();

        // 8. ROTATE SLIDES TO OUTTAKE POSITION
        while (opModeIsActive() && rotate.getPosition() <= rotate.outtakepos - 25) {
            rotate.setState("OUTTAKE");
            slides.setState("SLIDES RETRACTED", rotate);

        }
//

        // 9. DRIVE TO DELIVER SAMPLE 2
        drive.strafe_left(0.7, strafe_2);
        drive.rotate(0.7, rotate_3);
        drive.drive(0.7, drive_5);

        // 10. DELIVER SAMPLE 2
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
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+30){
            slides.setState("SLIDES RETRACTED", rotate);
        }
        slides.resetSLides();

        // NOT NEEDED
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < time_ms){
            slides.setState("SLIDES RETRACTED", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.rotate(0.7, rotate_4);
        drive.strafe_left(0.7, strafe_3);
        drive.drive(0.7, drive_6);
        rotate.resetRotation();
        while (opModeIsActive() && rotate.getPosition() >= rotate.intakepos + 25) {
            slides.setState("SLIDES RETRACTED", rotate);
            rotate.setState("INTAKE");
        }
        drive.intake();
        while (opModeIsActive() && slides.getPosition() <= slides.closeIntakePos){
            slides.setState("CLOSE INTAKE", rotate);
        }
        drive.drive(0.7, drive_8);
        slides.armSlideR.setPower(0);
        slides.armSlideL.setPower(0);
//        drive.drive(0.1, drive_7);
        drive.intakeStop();
        while (opModeIsActive() && rotate.getPosition() <= rotate.outtakepos - 25) {
            rotate.setState("OUTTAKE");
            slides.setState("SLIDES RETRACTED", rotate);

        }
        drive.drive(0.7, -drive_7);
        drive.strafe_left(0.7, -strafe_3);
        drive.rotate(0.7,rotate_5);
        drive.drive(0.7, drive_5);
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
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+30){
            slides.setState("SLIDES RETRACTED", rotate);
        }
        slides.resetSLides();


    }
}

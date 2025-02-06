package org.firstinspires.ftc.teamcode.auto.nonpushbotAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.StartServo;

@Config
@Autonomous(name="Left 3 Sample Faster")
public class Left3SampleFaster extends LinearOpMode {
    Drive drive;
    Slides slides;
    Rotate rotate;
    StartServo startServo;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer_full = new ElapsedTime();

    double distance;

    double ticks_per_degree = 10.825;
    public static int drive_01 = 5;
    public static int drive_02 = 54;
    public static int rotate_03 = -45;
    public static int drive_04 = -17;
    public static int rotate_05 = 64;
    public static int strafe_06 = 0;
    public static int drive_07 = -10;

    public static int drive_08 = 25;
    public static int drive_09 = -15;
    public static int rotate_10 = -65;
//    public static int rotate_10 = -45;
    public static int drive_11 = 0;
    public static int rotate_12 = 85;
    public static int strafe_13 = -40;
    public static int drive_14 = -10;
    public static int adjust_15 = -5;
    public static int drive_16 = 22;
    public static int drive_17 = -17;
    int rotate_18 = -55;
    public static int strafe_19 = -3;
    public static int drive_20 = -5;
    public static int adjust = -5;
    public static int rotate_21 = -20;
    public static int drive_22 = -2;
    public static int outtake_ms = 400;

    public static int outtake_adjust= 15;





    @Override
    public void runOpMode() {

        // INITIALIZE
        timer_full.reset();
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
        // Turn on the intake servos for a bit to make sure
        // that they work on the first outtake.  This makes not sense
        // but it fixed the problem.
        drive.intake();
        drive.drive(0.3, drive_01);
        drive.intakeStop();

        // 2. ROTATE ARM TO 0 POSITION
        rotate.armRotateL.setPower(0.55);
        rotate.armRotateR.setPower(0.55);
        sleep(500);
        rotate.armRotateL.setPower(-1);
        rotate.armRotateR.setPower(-1);
        sleep(40);
        rotate.armRotateL.setPower(0);
        rotate.armRotateR.setPower(0);

        // THIS SEEMED TO FIX THE ROTATION ISSUE. EACH TIME BEFORE YOU
        // ROTATE THE SLIDES TO THE INTAKE POSITION, RUN THESE FOUR LINES.
        // TO MAKE IT EASIER, MAKE A METHOD IN YOUR Rotate CLASS TO
        // RUN THESE FOUR LINES FOR YOU.  SOMETHING LIKE rezeroRotation()

        // RESET MOTOR MODES
        rotate.resetRotation();

        startServo.open();

        // 3. DRIVE TO LINE UP SAMPLE 1 DELIVERY
        drive.drive(0.825, drive_02);
        drive.rotate(0.5, rotate_03);
        drive.drive(0.825, drive_04);

        // 4. DELIVER SAMPLE 1
        slides.resetSLides();
        while (opModeIsActive() && slides.getPosition() <= slides.highOuttakePos - outtake_adjust){
            slides.setState("HIGH OUTTAKE", rotate);
        }
        drive.outtake();
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < outtake_ms){
            slides.setState("HIGH OUTTAKE", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.intakeStop();
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+25){
            slides.setState("SLIDES RETRACTED", rotate);
        }


//        // Not sure what this does
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < time_ms){
//            slides.setState("SLIDES RETRACTED", rotate);
//            dashboardTelemetry.addData("timer", timer);
//            dashboardTelemetry.update();
//        }

        // 5. DRIVE TO SAMPLE 2 INTAKE
        drive.rotate(0.825, rotate_05);
        drive.strafe_left(0.825, strafe_06);
        drive.drive(0.825, drive_07);

        // 6. ROTATE SLIDES TO INTAKE POSITION
        slides.resetSLides();
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

        // 7. INTAKE SAMPLE 2
        drive.intake();
        while (opModeIsActive() && slides.getPosition() <= slides.closeIntakePos){
            slides.setState("CLOSE INTAKE", rotate);
        }
        slides.armSlideR.setPower(0);
        slides.armSlideL.setPower(0);

        // SUGGESTION: Change this to slowly extend slides.
        drive.drive(0.15, drive_08);
        drive.intakeStop();
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+25){
            slides.setState("SLIDES RETRACTED", rotate);
        }

        // 8. ROTATE SLIDES TO OUTTAKE POSITION
        while (opModeIsActive() && rotate.getPosition() <= rotate.outtakepos - 25) {
            rotate.setState("OUTTAKE");
            slides.setState("SLIDES RETRACTED", rotate);
        }
        slides.resetSLides();
//

        // 9. DRIVE TO DELIVER SAMPLE 2
        drive.drive(0.825,drive_09);
        drive.rotate(0.825, rotate_10);
        drive.drive(0.825, drive_11);

         //10. DELIVER SAMPLE 2
        while (opModeIsActive() && slides.getPosition() <= slides.highOuttakePos-outtake_adjust){
            slides.setState("HIGH OUTTAKE", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < outtake_ms){
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
        drive.rotate(0.825, rotate_12);
        // 10. DELIVER SAMPLE 2

        // 11. DRIVE TO SAMPLE 3
        // 12. INTAKE SAMPLE 3
        rotate.resetRotation();
        while (opModeIsActive() && rotate.getPosition() >= rotate.intakepos + 25) {
            slides.setState("SLIDES RETRACTED", rotate);
            rotate.setState("INTAKE");
        }
        slides.resetSLides();
        drive.drive(0.825,adjust_15);
        drive.intake();
        while (opModeIsActive() && slides.getPosition() <= slides.closeIntakePos){
            slides.setState("CLOSE INTAKE", rotate);
        }
        slides.armSlideR.setPower(0);
        slides.armSlideL.setPower(0);

        drive.drive(0.1, drive_16);
        drive.intakeStop();

        // 13. ROTATE SLIDES TO OUTTAKE
        while (opModeIsActive() && rotate.getPosition() <= rotate.outtakepos - 25) {
            rotate.setState("OUTTAKE");
            slides.setState("SLIDES RETRACTED", rotate);
        }

        // 14. DRIVE TO OUTTAKE
//        drive.drive(0.825, drive_17);
//        drive.strafe_left(0.825, -strafe_13);
        drive.rotate(0.825, rotate_18);
//        drive.strafe_left(0.825, strafe_19);
        drive.drive(0.825, drive_20);
        drive.rotate(0.825, rotate_21);
        drive.drive(0.825, drive_22);

        // 15. OUTTAKE SAMPLE
        while (opModeIsActive() && slides.getPosition() <= slides.highOuttakePos-outtake_adjust){
            slides.setState("HIGH OUTTAKE", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < outtake_ms){
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

        dashboardTelemetry.addData("timer_full", timer_full.seconds());
    }
}

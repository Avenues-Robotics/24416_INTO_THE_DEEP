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
@Autonomous(name="Right 2 specimen")
public class Right2Specimen extends LinearOpMode {
    Drive drive;
    Slides slides;
    Rotate rotate;
    StartServo startServo;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer_full = new ElapsedTime();

    double distance;

    double ticks_per_degree = 10.8;

    public static double init_rotate_power = 0.6;
    public static int init_rotate_time = 300;
    public static int drive_01 = -54;
    public static int drive_03 = 30;

    public static int strafe_02 = 15;
    public static int rotate_04 = 90;

    public static int drive_05 = 60;

    public static int outtakespecimen_ms = 600;

    public static int outtake_adjust = 15;

    public static int drive_06 = -70;

    public static int rotate_07 = 45;

    public static int drive_08 = -20;

    public static int drive_02 = -10;





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

        // 1. DRIVE FORWARD
        // Turn on the intake servos for a bit to make sure
        // that they work on the first outtake.  This makes not sense
        // but it fixed the problem.
        drive.drive(0.3, drive_01);

        // 2. ROTATE ARM TO 0 POSITION
        rotate.armRotateL.setPower(init_rotate_power);
        rotate.armRotateR.setPower(init_rotate_power);
        sleep(init_rotate_time);
        rotate.armRotateL.setPower(0);
        rotate.armRotateR.setPower(0);
        sleep(500);

        // THIS SEEMED TO FIX THE ROTATION ISSUE. EACH TIME BEFORE YOU
        // ROTATE THE SLIDES TO THE INTAKE POSITION, RUN THESE FOUR LINES.
        // TO MAKE IT EASIER, MAKE A METHOD IN YOUR Rotate CLASS TO
        // RUN THESE FOUR LINES FOR YOU.  SOMETHING LIKE rezeroRotation()

        // RESET MOTOR MODES
        rotate.resetRotation();

        startServo.open();

        while (opModeIsActive() && slides.getPosition() <= rotate.outtakepos - 25){
            slides.setState("OUTTAKE", rotate);
            slides.setState("SLIDES RETRACTED",rotate);
        }


        // 4. DRIVE TO LINE UP SPECIMEN 1 DELIVERY
        drive.strafe_left(0.8,strafe_02);
        drive.drive(0.8,drive_02);

        // 5. DELIVER SPECIMEN 1
        slides.resetSLides();
        while (opModeIsActive() && slides.getPosition() <= slides.CloseIntakeAutoPos - outtake_adjust){
            slides.setState("CLOSE INTAKE", rotate);
        }
        drive.outtake();
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < outtakespecimen_ms){
            slides.setState("CLOSE INTAKE", rotate);
            dashboardTelemetry.addData("timer", timer);
            dashboardTelemetry.update();
        }
        drive.intakeStop();
        while (opModeIsActive() && slides.getPosition() >= slides.slidesRetractedPos+25){
            slides.setState("SLIDES RETRACTED", rotate);
        }

        // 6. DRIVE TO SPECIMEN 2 INTAKE
        drive.rotate(0.8, drive_03);
        drive.strafe_left(0.8, rotate_04);
        drive.drive(0.8, drive_05);

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

        // 7. INTAKE SPECIMEN 2
        drive.intake();
        while (opModeIsActive() && slides.getPosition() <= slides.closeIntakePos){
            slides.setState("CLOSE INTAKE", rotate);
        }
        slides.armSlideR.setPower(0);
        slides.armSlideL.setPower(0);

        // SUGGESTION: Change this to slowly extend slides.
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

        // 9. DRIVE TO DELIVER SPECIMEN 2
        drive.drive(0.8,drive_06);
        drive.rotate(0.8, rotate_07);
        drive.drive(0.8, drive_08);

        //10. DELIVER SPECIMEN 2
        while (opModeIsActive() && slides.getPosition() <= slides.CloseIntakeAutoPos-outtake_adjust){
            slides.setState("CLOSE INTAKE", rotate);
        }
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < outtakespecimen_ms){
            drive.outtake();
            slides.setState("CLOSE INTAKE", rotate);
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
package org.firstinspires.ftc.teamcode.auto.nonpushbotAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.StartServo;

@Config
@Autonomous(name="Right Auto 1234")
public class RightfirstnonpushbotAuto extends LinearOpMode {
    Drive drive;
    Slides slides;
    Rotate rotate;
    StartServo startServo;
    public static double ticks_per_degree = 10.72;
    public static int strafe_1 = 20;
    public static int rotate_1 = -90;
    public static int drive_1 = 30;
    public static int drive_2 = -30;
    public static int rotate_2 = -90;
    public static int drive_3 = 70;
    @Override
    public void runOpMode() {
        CRServo rServo = hardwareMap.get(CRServo.class, "rServo");
        CRServo lServo = hardwareMap.get(CRServo.class, "lServo");
        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);
        drive = new Drive(this);
        drive.TICKS_PER_DEGREE = ticks_per_degree;
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
//        startServo.open();
        drive.strafe_left(0.3, strafe_1);
        drive.rotate(0.3, rotate_1);
        drive.drive(0.5, drive_1);
        drive.drive(0.5, drive_2);
        drive.rotate(0.3, rotate_2);
        drive.drive(0.5, drive_3);






    }
}

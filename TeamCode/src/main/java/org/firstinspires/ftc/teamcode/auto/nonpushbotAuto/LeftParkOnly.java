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
@Autonomous(name="Left Park Only")
public class LeftParkOnly extends LinearOpMode {
    Drive drive;
    Slides slides;
    Rotate rotate;
    StartServo startServo;

    @Override
    public void runOpMode() {
        CRServo rServo = hardwareMap.get(CRServo.class, "rServo");
        CRServo lServo = hardwareMap.get(CRServo.class, "lServo");
        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);
        drive = new Drive(this);
//        slides = new Slides(this);
//        rotate = new Rotate(this);
//        startServo = new StartServo(this);
//        startServo.start();
        waitForStart();
        drive.drive(0.5, 20);
//        rotate.armRotateL.setPower(0.5);
//        rotate.armRotateR.setPower(0.5);
//        sleep(300);
//        rotate.armRotateL.setPower(0);
//        rotate.armRotateR.setPower(0);
//        startServo.open();
//        sleep(1000);
//        drive.drive(0.5,80);
//        drive.drive(0.5, -65);
//        drive.Strafe_left(0.5, 10);
//        drive.rotate(0.5,45);
//        rotate.setState("OUTTAKE");
//        sleep(1000);
//        slides.setState("HIGH OUTTAKE", rotate);
//        sleep(2000);
//        drive.outtake();
//        slides.setState("SLIDES RETRACTED", rotate);
//        rotate.setState("INTAKE");
//        drive.rotate(0.5, -45);
//        drive.Strafe_left(0.5, 10);
//        drive.drive(0.5,50);
//        drive.drive(0.5, -100);






    }
}

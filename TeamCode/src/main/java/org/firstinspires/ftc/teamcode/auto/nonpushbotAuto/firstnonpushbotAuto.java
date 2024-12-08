package org.firstinspires.ftc.teamcode.auto.nonpushbotAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;

@Config
@Autonomous(name="RyderLiouAutoPushbotLeft {:")
public class firstnonpushbotAuto extends LinearOpMode {
    Drive drive;
    Slides slides;
    Rotate rotate;

    @Override
    public void runOpMode() {
        CRServo rServo = hardwareMap.get(CRServo.class, "rServo");
        CRServo lServo = hardwareMap.get(CRServo.class, "lServo");
        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);
        drive = new Drive(this);
        slides = new Slides(this);
        rotate = new Rotate(this);
        waitForStart();
        rotate.setState("INTAKE");
        drive.Strafe_left(0.5, 50);
        drive.drive(0.5, 50);
        drive.Intake();
        drive.rotate(0.5, 30);
        rotate.setState("OUTTAKE");
        drive.Outtake();
        slides.setState("SLIDES OUTTAKE", rotate.getPosition());
        drive.rotate(0.5, -30);
        drive.drive(0.5, 20);
        drive.Strafe_right(0.5, 20);





    }
}

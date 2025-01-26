package org.firstinspires.ftc.teamcode.auto.nonpushbotAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.StartServo;

@Config
@Autonomous(name="Only Rotate")
@Disabled
public class OnlyRotate extends LinearOpMode {
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
        slides = new Slides(this);
        rotate = new Rotate(this);
        startServo = new StartServo(this);
        startServo.start();
        waitForStart();
        rotate.armRotateL.setPower(0.5);
        rotate.armRotateR.setPower(0.5);
        sleep(300);
        rotate.armRotateL.setPower(0);
        rotate.armRotateR.setPower(0);
        startServo.open();
        sleep(1000);

    }
}

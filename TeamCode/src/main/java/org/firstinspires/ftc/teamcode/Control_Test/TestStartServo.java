package org.firstinspires.ftc.teamcode.Control_Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.StartServo;

@Config
@Autonomous(name="Test Start Servo")
public class TestStartServo extends LinearOpMode {

    StartServo startServo;
    Rotate rotate;
    public static double startRotatePower = 1;
    public static int startTime = 10;

    public void runOpMode() {

        startServo = new StartServo(this);
        rotate = new Rotate(this);
        startServo.start();

        waitForStart();

        rotate.armRotateL.setPower(0.5);
        rotate.armRotateR.setPower(0.5);
        sleep(300);
        rotate.armRotateL.setPower(0);
        rotate.armRotateR.setPower(0);
        startServo.open();

        while(opModeIsActive()){

        }

    }

}

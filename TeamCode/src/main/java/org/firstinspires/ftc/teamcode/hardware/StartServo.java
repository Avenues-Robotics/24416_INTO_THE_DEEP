package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class StartServo {

    LinearOpMode opMode;
    Servo servo;
    public static double startPosition = 0;
    public static double openPosition = .2;

    public StartServo(LinearOpMode opModeCalledFrom){
       opMode = opModeCalledFrom;
       servo = opMode.hardwareMap.get(Servo.class, "startServo");
    }

    public void start(){
        servo.setPosition(startPosition);
    }

    public void open(){
        servo.setPosition(openPosition);
    }

}

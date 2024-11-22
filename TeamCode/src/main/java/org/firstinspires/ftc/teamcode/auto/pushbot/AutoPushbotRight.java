package org.firstinspires.ftc.teamcode.auto.pushbot;


/***********************************************************************
 *
 * Pushbot: Autonomous Right
 *
 * Scores 5 points
 * In this OpMode the robot starts on the RIGHT, scores a sample in the
 * NET ZONE, and parks in the OBSERVATION ZONE while avoiding a stationary
 * alliance partner robot.
 *
 * The robot should start ... (YOU DECIDE HOW THE ROBOT SHOULD BE
 * POSITIONED)
 ***********************************************************************/
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.utilities.PIDF;


@Config
@Autonomous(name="RyderLiouAutoPushbotRight {:")
public class AutoPushbotRight extends LinearOpMode {
    Drive drive;

    @Override
    public void runOpMode() {

        drive = new Drive(this);

//        PIDF drivePIDF = new PIDF(.5, .1, .1, 0, 3);
//        PIDF slidePIDF = new PIDF(.3, .1, .05, 10, 3);
//
//        waitForStart();
//
//        while(something){
//            double targetdistance = 100.;
//            double distanceToTarget = distancesensorvalue;
//
//            double motorPower = drivePIDF.update(targetdistance, distanceToTarget);
//
//            motor.setPower(motorPower);
//        }


        drive.drive(1, 110);
        drive.drive(1, -275);


    }
}
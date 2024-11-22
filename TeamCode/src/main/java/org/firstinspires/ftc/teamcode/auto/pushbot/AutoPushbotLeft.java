package org.firstinspires.ftc.teamcode.auto.pushbot;

/***********************************************************************
 *
 * Pushbot: Autonomous Left
 *
 * Scores 5 points
 * In this OpMode the robot starts on the LEFT, scores a sample in the
 * NET ZONE, and parks in the OBSERVATION ZONE.
 *
 * The robot should start facing the NET ZONE with a SAMPLE touching
 * the front cross bar of the robot so it can push it straight into
 * the NET ZONE.
 ***********************************************************************/
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drive;


@Config
@Autonomous(name="RyderLiouAutoPushbotLeft {:")
public class AutoPushbotLeft extends LinearOpMode {
    Drive drive;

    @Override
    public void runOpMode() {

        drive = new Drive(this);
        waitForStart();
        drive.drive(1, 110);
        drive.drive(1, -275);


    }
}

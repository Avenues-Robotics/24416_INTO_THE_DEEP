/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Linear OpMode")
public class Teleop extends LinearOpMode {
    Gamepad prevGamepad1 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    Slides slides;
    String slidesState;
    String rotateState;
    Rotate rotate;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // The motor members could be initialized using new Drive(this),
        // Then you wouldn't need to do it all here.  Keeps the code cleaner.
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");
        CRServo rServo = hardwareMap.get(CRServo.class, "rServo");
        CRServo lServo = hardwareMap.get(CRServo.class, "lServo");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        lServo.setDirection(CRServo.Direction.FORWARD);
        rServo.setDirection(CRServo.Direction.REVERSE);

        slides = new Slides(this);
        slidesState = "SLIDES RETRACTED";

        rotate = new Rotate(this);
        rotateState = "OUTTAKE";

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        // Control Loop
        while (opModeIsActive()) {

            // Set the value of prevGamepad1/2 to the value of from the previous loop
            prevGamepad1.copy(currentGamepad1);
            prevGamepad2.copy(currentGamepad2);

            // Set the value of currentGamepad1/2 to the current state of the gamepad
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // run until the end of the match (driver presses STOP)

            // It would be cleaner to move all of this code to the Drive class
            // Just make a method like Drive.stickDrive() and all of this code
            // could go there.  Just remember that gamepad1 would be opMode.gamepad1

            // DRIVE
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad2.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad2.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad2.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad2.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            if(currentGamepad1.right_trigger >= 0.5) {
                FL.setPower(leftFrontPower);
                FR.setPower(rightFrontPower);
                BL.setPower(leftBackPower);
                BR.setPower(rightBackPower);
            }

            if(currentGamepad1.right_trigger < 0.5) {
                FL.setPower(leftFrontPower / 3.5);
                FR.setPower(rightFrontPower / 3.5);
                BL.setPower(leftBackPower / 3.5);
                BR.setPower(rightBackPower / 3.5);
            }


            // END DRIVE

            // SLIDES

            if (currentGamepad2.dpad_down && !prevGamepad2.dpad_down) {
                slidesState = "SLIDES RETRACTED";
            }
            if (currentGamepad2.dpad_left && !prevGamepad2.dpad_left) {
                slidesState = "CLOSE INTAKE";
            }
            if (currentGamepad2.dpad_right && !prevGamepad2.dpad_right) {
                slidesState = "LOW OUTTAKE";
            }
            if (currentGamepad2.dpad_up && !prevGamepad2.dpad_up) {
                slidesState = "HIGH OUTTAKE";
            }
            if (Math.abs(currentGamepad2.left_stick_x) >= 0.1) {
                slidesState = "MANUAL";
            }
            if (slidesState.equals("MANUAL") && Math.abs(currentGamepad2.left_stick_x) < 0.1) {
                slidesState = "HOLD";
                slides.setTargetPosition(slides.getPosition());
            }
            slides.setState(slidesState, rotate);

            // ROTATE
            if ((currentGamepad2.x && !prevGamepad2.x) && (slides.getPosition() < 50)) {
                // THIS ALSO USES x
                rotateState = "OUTTAKE";
            }
            if ((currentGamepad2.b && !prevGamepad2.b) && (slides.getPosition() < 50)) {
                rotateState = "INTAKE";
            }

            rotate.setState(rotateState);

            // INTAKE SERVOS
            // INTAKE
            if ((currentGamepad2.right_bumper)) {
                lServo.setPower(1);
                rServo.setPower(1);
            }
            // OUTTAKE
            else if ((currentGamepad2.left_bumper)) {
                lServo.setPower(-1);
                rServo.setPower(-1);
            } else {
                lServo.setPower(0);
                rServo.setPower(0);
            }

            // TELEMETRY

            telemetry.addLine("SLIDES");
            telemetry.addData("state", slidesState);
            telemetry.addData("target", slides.targetPosition);
            telemetry.addData("position", slides.getPosition());
            telemetry.addData("power", "R:" + slides.armSlideR.getPower() + "| L:" + slides.armSlideL.getPower());
            telemetry.addLine("");
            telemetry.addLine("ROTATE");
            telemetry.addData("state", rotateState);
            telemetry.addData("target position", rotate.targetPosition);
            telemetry.addData("position", rotate.getPosition());
            telemetry.addData("power", "R:" + rotate.armRotateR.getPower() + "| L:" + rotate.armRotateL.getPower());
            telemetry.addData("x", currentGamepad2.x);
            telemetry.addData("y", currentGamepad2.y);
            telemetry.addData("BL position", BL.getCurrentPosition());
            telemetry.addData("FR position", FR.getCurrentPosition());
            telemetry.addData("BR position", BR.getCurrentPosition());
            telemetry.addData("FL position", FL.getCurrentPosition());

            telemetry.update();

        }


    }
}
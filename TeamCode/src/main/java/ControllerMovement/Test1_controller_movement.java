

package ControllerMovement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Controller Movement Test")
@Disabled
public class Test1_controller_movement extends LinearOpMode {

    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor FL = null;

    @Override
    public void runOpMode() {
        FL  = hardwareMap.get(DcMotor.class, "left_front_drive");
        FR  = hardwareMap.get(DcMotor.class, "left_back_drive");
        BL = hardwareMap.get(DcMotor.class, "right_front_drive");
        BR = hardwareMap.get(DcMotor.class, "right_back_drive");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  = max;
                rightFrontPower = max;
                leftBackPower   = max;
                rightBackPower  = max;
            }

            FL.setPower(leftFrontPower);
            FR.setPower(rightFrontPower);
            BL.setPower(leftBackPower);
            BR.setPower(rightBackPower);
            telemetry.addData("FLpower", leftFrontPower);
            telemetry.addData("FRpower", rightFrontPower);
            telemetry.addData("BLpower", leftBackPower);
            telemetry.addData("BRpower", rightBackPower);
            telemetry.update();
        }
    }
}
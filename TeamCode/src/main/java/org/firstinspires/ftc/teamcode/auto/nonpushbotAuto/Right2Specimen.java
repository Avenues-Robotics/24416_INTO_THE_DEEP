package org.firstinspires.ftc.teamcode.auto.nonpushbotAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Rotate;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.StartServo;

@Config
public class Right2Specimen extends LinearOpMode {
    Drive drive;
    Slides slides;
    Rotate rotate;
    StartServo startServo;
    ElapsedTime timer = new ElapsedTime();
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
        rotate.armRotateL.setPower(0);
        rotate.armRotateR.setPower(0);
        startServo.open();
        }
        drive.outtake();
        }
        drive.intakeStop();
            slides.setState("SLIDES RETRACTED", rotate);
        }


        slides.resetSLides();
        while (opModeIsActive() && slides.getPosition() <= slides.closeIntakePos){
            slides.setState("CLOSE INTAKE", rotate);
        }
        slides.armSlideR.setPower(0);
        slides.armSlideL.setPower(0);
        drive.intakeStop();
            slides.setState("SLIDES RETRACTED", rotate);
        }
        slides.resetSLides();
        }
            drive.outtake();
        }
        drive.intakeStop();

   }
}
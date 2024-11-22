package Control_Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.PIDF;

import java.util.Objects;

@Config
@Autonomous(name="Ryder Linear slide test")
public class ArmRotation_prediction extends LinearOpMode {
    DcMotor armRotateR;
    DcMotor armRotateL;

    PIDF intakePIDF;
    PIDF outakePIDF;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    //adjust the Kp Ki Kf Kd to diffrent things based opon if it needs to go forward or back

    String state;
    public static double targetPosition = 0;

    public static double intake_Kp = 0;
    public static double intake_Ki = 0;
    public static double intake_Kd = 0;
    public static double intake_Kf = 0;

    public static double outtake_Kp = 0;
    public static double outtake_Ki = 0;
    public static double outtake_Kd = 0;
    public static double outtake_Kf = 0;

    public static double tolerance = 5;


    int pos0 = -4; // INTAKE
    int pos1 = -2;
    int pos2 = 0; // START
    int pos3 = 6;
    int pos4 = 8;
    int pos5 = 10;
    int pos6 = 12;
    int pos7 = 14;
    int pos8 = 16; // OUTAKE


    public void runOpMode() {
        // Initialization arm motors
        armRotateR = hardwareMap.get(DcMotor.class, "ArmSlideR");
        armRotateL = hardwareMap.get(DcMotor.class, "ArmSlideL");
        armRotateR.setDirection(DcMotor.Direction.REVERSE);
        armRotateL.setDirection(DcMotor.Direction.FORWARD);
        armRotateR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotateL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotateR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotateL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePIDF = new PIDF(intake_Kp, intake_Ki, intake_Kd, intake_Kf, tolerance);
        outakePIDF = new PIDF(outtake_Kp, outtake_Ki, outtake_Kd, outtake_Kf, tolerance);

        while(opModeInInit()){
            dashboardTelemetry.addData("motor rotate position", armRotateR.getCurrentPosition());
            dashboardTelemetry.addData("motor rotate position", armRotateL.getCurrentPosition());
            dashboardTelemetry.update();
        }

        waitForStart();

        double intakePower;
        // Control Loop
        double outtakePower;

        while (opModeIsActive()) {

            int currentValue = (armRotateR.getCurrentPosition() + armRotateL.getCurrentPosition()) / 2;

            //repeat for all possible values of the 9 positions test to find the kps
            if(pos0 < currentValue && currentValue < pos1 && Objects.equals(state, "OUTAKE")){
               //set a value that holds it in place to a degree/slows it down
                outtake_Kf = 0;
            }
            if(pos0 < currentValue && currentValue< pos1 && Objects.equals(state, "INTAKE")){
                //set a value that holds it in place to a degree/slows it down
                intake_Kf = 0;
            }

            // eventually use gamepad to set state
            // eventually use gamepad to set state
            // eventually use gamepad to set state
            // eventually use gamepad to set state
            // eventually use gamepad to set state
            state = "INTAKE";

            if(state == "START"){
                outtakePower = 0;//put the tick value of the "hammer" position.;

            }
            else if(state == "OUTTAKE"){
                targetPosition = 0;//put the tick value of the "outtake" position;
                intakePower = outakePIDF.update(targetPosition,currentValue);

            }


            if(state == "INTAKE"){
                targetPosition = 0;//put the tick value of the intake position;
                outtakePower = intakePIDF.update(targetPosition, currentValue);
            }

            }
            armRotateR.setPower(outtakePower);
            armRotateL.setPower(intakePower);
            dashboardTelemetry.addData("motor left position", currentValue);
            dashboardTelemetry.update();
    }

    }


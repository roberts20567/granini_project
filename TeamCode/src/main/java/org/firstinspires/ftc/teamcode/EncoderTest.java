package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Test Encodere")
public class EncoderTest extends LinearOpMode{

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private DcMotor motorRidicare;
    private DcMotor motorIntindere;
    private DcMotor motorBrat;

    private int target = 1267;



    @Override
    public void runOpMode() throws InterruptedException {

        motorBackLeft = hardwareMap.dcMotor.get("motor_test_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_2");
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_3");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_4");
        motorRidicare = hardwareMap.dcMotor.get("motor_lift");
        motorIntindere = hardwareMap.dcMotor.get("motor_nebun");
        motorBrat = hardwareMap.dcMotor.get("motor_ridicare");


        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRidicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntindere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRidicare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontRight.setTargetPosition(target);
        motorBackLeft.setTargetPosition(target);
        motorFrontLeft.setTargetPosition(target);
        motorBackRight.setTargetPosition(target);
        motorRidicare.setTargetPosition(target);
        motorIntindere.setTargetPosition(target);
        motorBrat.setTargetPosition(target);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Position BL:" ,motorBackLeft.getCurrentPosition());
            telemetry.addData("Position BR:" ,motorBackRight.getCurrentPosition());
            telemetry.addData("Position FL:" ,motorFrontLeft.getCurrentPosition());
            telemetry.addData("Position FR:" ,motorFrontRight.getCurrentPosition());
            telemetry.addData("Position Lift:" ,motorRidicare.getCurrentPosition());
            telemetry.addData("Position Intindere:" ,motorIntindere.getCurrentPosition());
            telemetry.addData("Position Brat:" ,motorBrat.getCurrentPosition());
            telemetry.update();

        }

    }

}

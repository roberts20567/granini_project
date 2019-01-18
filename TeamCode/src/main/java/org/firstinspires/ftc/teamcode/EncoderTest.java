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

    private int target = 1267;



    @Override
    public void runOpMode() throws InterruptedException {

        motorBackLeft = hardwareMap.dcMotor.get("motor_test_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_2");
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_3");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_4");

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontRight.setTargetPosition(target);
        motorBackLeft.setTargetPosition(target);
        motorFrontLeft.setTargetPosition(target);
        motorBackRight.setTargetPosition(target);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            motorBackLeft.setPower(0.4);
            motorFrontRight.setPower(0.4);
            motorBackRight.setPower(0.4);
            motorFrontLeft.setPower(0.4);
            telemetry.addData("Position BL:" ,motorBackLeft.getCurrentPosition());
            telemetry.addData("Position BR:" ,motorBackRight.getCurrentPosition());
            telemetry.addData("Position FL:" ,motorFrontLeft.getCurrentPosition());
            telemetry.addData("Position FR:" ,motorFrontRight.getCurrentPosition());
            telemetry.update();

        }

    }

}

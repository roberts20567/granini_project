package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name = "Xeo18-19: Tele-OP", group = "Xeo18-19")
public class tele_op_xeo_alfa extends OpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorRidicare;
    private DcMotor motorHex;
    private double motor_power;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        motorRidicare = hardwareMap.dcMotor.get("motor_test_5");
        motorHex = hardwareMap.dcMotor.get("motor_hex");
        motor_power = 0.5;
    }

    @Override
    public void loop() {
        float gamepad1LeftY = -gamepad1.left_stick_y ;
        float gamepad1LeftX = -gamepad1.left_stick_x ;
        float gamepad1RightX = gamepad1.right_stick_x;

        // holonomic formulas
        float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        // clip the right/left values so that the values never exceed +/- 1
        FrontRight = Range.clip(FrontRight,  -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);
        if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left) {
            motorFrontRight.setPower(FrontRight * motor_power);
            motorFrontLeft.setPower(FrontLeft * motor_power);
            motorBackLeft.setPower(BackLeft * motor_power);
            motorBackRight.setPower(BackRight * motor_power);
        }
        /*if (gamepad1.a) {

            motorTest.setPower(1);

        }
        else {

            motorTest.setPower(0);

        }*/
        motorRidicare.setPower(gamepad2.left_stick_y);
        motorHex.setPower(gamepad2.y? 1:0);


    }

    @Override
    public void stop(){
        stopMotors();
        stopServos();
    }

    private void stopMotors() {

        motorBackRight.close();
        motorBackLeft.close();
        motorFrontRight.close();
        motorFrontLeft.close();
    }

    private void stopServos() {

    }

}

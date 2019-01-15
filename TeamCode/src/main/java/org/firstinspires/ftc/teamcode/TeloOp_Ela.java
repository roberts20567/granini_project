package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Xeo18-19: Ela se intoarce", group = "Xeo18-19")
public class TeloOp_Ela extends OpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private Servo servoButtomRight;
    private Servo servoButtomLeft;
    private Servo servoTopRight;
    private Servo servoTopLeft;

    private double motorPower = 0.3;
    double standard  = 0.3;
    private double servoPoz = 0.68;
    private double servoPozS = 0.32;
    private double servoPozD = 0.62;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");

        servoButtomLeft = hardwareMap.servo.get("servo_buttom_left");
        servoButtomRight = hardwareMap.servo.get("servo_buttom_right");
        servoTopLeft = hardwareMap.servo.get("servo_top_left");
        servoTopRight = hardwareMap.servo.get("servo_top_right");
    }

    @Override
    public void loop() {
        float gamepad1LeftY = -gamepad1.left_stick_y ;
        float gamepad1LeftX = -gamepad1.left_stick_x ;
        float gamepad1RightX = gamepad1.right_stick_x;

        float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        // clip the right/left values so that the values never exceed +/- 1
        FrontRight = Range.clip(FrontRight, -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);

        if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left)
        {
            motorFrontRight.setPower(FrontRight * motorPower);
            motorFrontLeft.setPower(FrontLeft * motorPower);
            motorBackLeft.setPower(BackLeft * motorPower);
            motorBackRight.setPower(BackRight * motorPower);
        }

        //cod fata spate dreapta stanga
        if (gamepad1.dpad_up)
        {
            motorFrontRight.setPower(motorPower);
            motorFrontLeft.setPower(-motorPower);
            motorBackLeft.setPower(-motorPower);
            motorBackRight.setPower(motorPower);
        }
        if (gamepad1.dpad_down)
        {
            motorFrontRight.setPower(-motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(motorPower);
            motorBackRight.setPower(-motorPower);
        }
        if (gamepad1.dpad_right)
        {
            motorFrontRight.setPower(motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(-motorPower);
            motorBackRight.setPower(-motorPower);
        }
        if (gamepad1.dpad_left)
        {
            motorFrontRight.setPower(-motorPower);
            motorFrontLeft.setPower(-motorPower);
            motorBackLeft.setPower(motorPower);
            motorBackRight.setPower(motorPower);
        }

        servoButtomRight.setPosition(servoPozD);
        servoButtomLeft.setPosition(servoPozS);
        servoTopRight.setPosition(servoPozD);
        servoTopLeft.setPosition(servoPozS);

        if (gamepad1.a) { // inchidere
            servoPozS = 0.26;
            servoPozD = 0.67;
        }
        if (gamepad1.b) { // deschidere
            servoPozS = 0.4614;
            servoPozD = 0.4785;
        }
        if (gamepad1.x) // deschis si mai tare
        {
            servoPozS = 0.75;
            servoPozD = 0.25;
        }


        if (gamepad1.left_trigger != 0 && servoPozD > 0.4 && servoPozS < 0.5) {
            servoPozS += gamepad2.left_trigger / 25;
            servoPozD -= gamepad2.left_trigger / 25;
        }
        if (gamepad1.right_trigger != 0 && servoPozS > 0.25 && servoPozD < 0.7) {
            servoPozS -= gamepad2.right_trigger / 25;
            servoPozD += gamepad2.right_trigger / 25;
        }



    }
}

package org.firstinspire .eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Xeo18-19: Jucati fotbal in plm", group = "Xeo18-19")
public class Football_teleop extends OpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private double motorPower=0.3;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
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
    }
}

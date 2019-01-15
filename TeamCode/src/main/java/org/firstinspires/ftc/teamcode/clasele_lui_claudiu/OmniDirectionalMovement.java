package org.firstinspires.ftc.teamcode.clasele_lui_claudiu;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class OmniDirectionalMovement {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private double motor_power = 0.5;
    private boolean pow = false;


    OmniDirectionalMovement(DcMotor motor_1, DcMotor motor_2, DcMotor motor_3, DcMotor motor_4){
        motorFrontRight = motor_1;
        motorFrontLeft = motor_2;
        motorBackLeft = motor_3;
        motorBackRight = motor_4;
    }



    private void initEncoders(){
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

/*
    void move(double left_stick_y, double left_stick_x, double right_stick_x){
        double gamepad1LeftY = -left_stick_y ;
        double gamepad1LeftX = -left_stick_x ;
        double gamepad1RightX = -right_stick_x;

        if(pow) {
            double angle = Math.toRadians(currentAngle - calibrationAngle);
            double motorX = Math.cos(angle) * gamepad1LeftX - Math.sin(angle) * gamepad1LeftY;
            double motorY = Math.cos(angle) * gamepad1LeftY + Math.sin(angle) * gamepad1LeftX;
            gamepad1LeftX = motorX;
            gamepad1LeftY = motorY;
        }

        double front_left = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        double front_right = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        double back_right = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        double back_left = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        // clip the right/left values so that the values never exceed +/- 1
        front_right = Range.clip(front_right,  -1, 1);
        front_left = Range.clip(front_left, -1, 1);
        back_left = Range.clip(back_left, -1, 1);
        back_right = Range.clip(back_right, -1, 1);

        motorFrontRight.setPower(front_right * motorPower);
        motorFrontLeft.setPower(front_left * motorPower);
        motorBackLeft.setPower(back_left * motorPower );
        motorBackRight.setPower(back_right * motorPower);
    }*/

}

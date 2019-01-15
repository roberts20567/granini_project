package org.firstinspires.ftc.teamcode.clasele_lui_claudiu;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class NClaudiuOmniDirectionalMovement {
    private DcMotor motor_front_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_right;
    private DcMotor motor_back_left;
    private Gamepad gamepad;
    private double motor_power;
    private double current_angle;
    private boolean pow_active;

    NClaudiuOmniDirectionalMovement(){
        motor_power = 0.66;
        current_angle = 0;
        pow_active = false;
        gamepad = null;
    }

    public void loopMove(){
        float gamepap_left_y = -gamepad.left_stick_y ;
        float gamepad_left_x = -gamepad.left_stick_x ;
        float gamepad_right_x = -gamepad.right_stick_x;

        if(pow_active) {
            //double angle = Math.toRadians(angles.firstAngle - start_angle);
            double angle = current_angle;
            double motorX = Math.cos(angle) * gamepad_left_x - Math.sin(angle) * gamepap_left_y;
            double motorY = Math.cos(angle) * gamepap_left_y + Math.sin(angle) * gamepad_left_x;
            gamepad_left_x = (float) motorX;
            gamepap_left_y = (float) motorY;
        }
        float FrontLeft = -gamepap_left_y - gamepad_left_x - gamepad_right_x;
        float FrontRight = gamepap_left_y - gamepad_left_x - gamepad_right_x;
        float BackRight = gamepap_left_y + gamepad_left_x - gamepad_right_x;
        float BackLeft = -gamepap_left_y + gamepad_left_x - gamepad_right_x;

        // clip the right/left values so that the values never exceed +/- 1
        FrontRight = Range.clip(FrontRight,  -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);

        motor_front_right.setPower(FrontRight * motor_power);
        motor_front_left.setPower(FrontLeft * motor_power);
        motor_back_right.setPower(BackLeft * motor_power);
        motor_back_left.setPower(BackRight * motor_power);
    }

    public void attachMotors(DcMotor motor_front_right, DcMotor motor_front_left, DcMotor motor_back_right, DcMotor motor_back_left){
        this.motor_front_right = motor_front_right;
        this.motor_front_left = motor_front_left;
        this.motor_back_right = motor_back_right;
        this.motor_back_left = motor_back_left;
    }

    public void dettachMotors(){
        motor_front_right = null;
        motor_front_left = null;
        motor_back_right = null;
        motor_back_left = null;
    }

    private void setMode(DcMotor.RunMode mode){
        motor_front_right.setMode(mode);
        motor_front_left.setMode(mode);
        motor_back_right.setMode(mode);
        motor_back_left.setMode(mode);
    }

    public double getMotorPower(){
        return motor_power;
    }

    public void setMotorPower(double motor_power){
        this.motor_power = Range.clip(motor_power, -1, 1);
    }

    public Gamepad getGamepad() {
        return gamepad;
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

}

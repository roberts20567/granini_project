package org.firstinspires.ftc.teamcode.clasele_lui_claudiu;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class NClaudiuOmniDirectionalMovementPro {
    private DcMotor[] dcMotorsArray;
    private int motorNumber = 0;
    private double[] xCoefficients;
    private double[] yCoefficients;
    private Gamepad gamepad = null;
    private double motorsPower = 0.5;
    private double robotFrontSideAngle = 0;
    private boolean POW = false;
    private double currentRobotAngle;
    private double calibrationAngle;

    public NClaudiuOmniDirectionalMovementPro(){

    }

    private void calculateCoefficients(){
        for(int i=0; i<motorNumber; i++){
            double motor_angle = 360 / motorNumber * i + robotFrontSideAngle;
            xCoefficients[i] = Math.cos(Math.toRadians(motor_angle));
            yCoefficients[i] = Math.sin(Math.toRadians(motor_angle));
        }
    }

    public void opModeLoop(){
        double gamepad_left_y = gamepad.left_stick_y ;
        double gamepad_left_x = gamepad.left_stick_x ;
        double gamepad_right_x = gamepad.right_stick_x;

        if(POW){
            double angle = currentRobotAngle - calibrationAngle;
            double aux_x = Math.cos(angle) * gamepad_left_x - Math.sin(angle) * gamepad_left_y;
            double aux_y = Math.cos(angle) * gamepad_left_y + Math.sin(angle) * gamepad_left_x;
            gamepad_left_x = aux_x;
            gamepad_left_y = aux_y;
        }

        for(int i=0; i<motorNumber; i++){
            double power = gamepad_left_x * xCoefficients[i] + gamepad_left_y * yCoefficients[i] + gamepad_right_x;
            dcMotorsArray[i].setPower(power * motorsPower);
        }

    }

    public void turnOnPOW(double calibrationAngle){
        POW = true;
        this.calibrationAngle = calibrationAngle;
    }

    public void turnOffPOW(){
        POW = false;
    }

    public void attachMotor(DcMotor motor){
        dcMotorsArray[motorNumber++] = motor;
    }

    public DcMotor[] getDcMotorsArray(){
        return dcMotorsArray;
    }

    public Gamepad getGamepad() {
        return gamepad;
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public double getMotorsPower() {
        return motorsPower;
    }

    public void setMotorsPower(double motorsPower) {
        this.motorsPower = motorsPower;
    }

    public double getRobotFrontSideAngle() {
        return robotFrontSideAngle;
    }

    public void setRobotFrontSideAngle(double robotFrontSideAngle) {
        this.robotFrontSideAngle = robotFrontSideAngle;
    }

    public void setCurrentRobotAngle(double currentRobotAngle) {
        this.currentRobotAngle = currentRobotAngle;
    }
}

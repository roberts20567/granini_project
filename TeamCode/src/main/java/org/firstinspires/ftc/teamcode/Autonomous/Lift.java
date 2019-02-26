package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

class Lift{
    private DcMotor motorLift1;
    private DcMotor motorLift2;

    Lift(DcMotor motor_1, DcMotor motor_2){
        motorLift1 = motor_1;
        motorLift2 = motor_2;
    }

    void setPower(double power){
        motorLift1.setPower(power);
        motorLift2.setPower(-power);
    }

    void setTargetPOsition(int target){
        motorLift1.setTargetPosition(target);
        motorLift2.setTargetPosition(-target);
    }

    public DcMotor getMotorLift1() {
        return motorLift1;
    }

    public void setMotorLift1(DcMotor motorLift1) {
        this.motorLift1 = motorLift1;
    }

    public DcMotor getMotorLift2() {
        return motorLift2;
    }

    public void setMotorLift2(DcMotor motorLift2) {
        this.motorLift2 = motorLift2;
    }
}
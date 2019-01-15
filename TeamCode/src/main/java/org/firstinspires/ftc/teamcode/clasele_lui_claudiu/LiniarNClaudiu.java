package org.firstinspires.ftc.teamcode.clasele_lui_claudiu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class LiniarNClaudiu extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private BNO055IMU imu;
    private Orientation angles;

    public void Init(){
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
    }

    public void InitGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void goFront(double speed){
        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft. setPower(-speed);
    }

    public void goBack(double speed){
        goFront(-speed);
    }

    public void goRight(double speed){
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(-speed);
        motorBackLeft. setPower(-speed);
    }

    public void goLeft(double speed){
        goRight(-speed);
    }

    public void rotateClockwise (double speed){
        motorFrontLeft.setPower(speed);
        motorBackRight.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
    }

    public void rotateAntiClockwise(double speed){
        rotateClockwise(-speed);
    }

}

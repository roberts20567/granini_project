package org.firstinspires.ftc.teamcode.clasele_lui_claudiu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class NClaudiu extends OpMode {
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;

    public BNO055IMU imu;
    public Orientation angles;

    private void initMovement(){
        this.motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        this.motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        this.motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        this.motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
    }

    private void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
    }

    public abstract void myInit();
    public abstract void myLoop();

    @Override
    final public void init(){
        initMovement();
        initGyro();
        myInit();
    }

    @Override
    final public void loop(){
        myLoop();
    }

}

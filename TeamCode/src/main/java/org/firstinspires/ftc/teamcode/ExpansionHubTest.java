package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name="REV Test", group = "Sensors")
public class ExpansionHubTest extends LinearOpMode {

    DcMotor motortest;
    BNO055IMU imu;
    Orientation angles;
    Sensor sensorTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        motortest = hardwareMap.dcMotor.get("motor_test");
        motortest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motortest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sensorTelemetry = new Sensor(telemetry);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        sensorTelemetry.initSensor(imu);

        /*        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();*/

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        /* imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters); */



        while (!opModeIsActive() && !Thread.currentThread().isInterrupted()) {

            telemetry.addLine("Start me please");
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading:" ,formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("roll:" ,formatAngle(angles.angleUnit, angles.secondAngle));
            telemetry.addData("pitch:" ,formatAngle(angles.angleUnit, angles.thirdAngle));
            telemetry.update();

        }

        while (opModeIsActive()) {
            motortest.setTargetPosition(1440);
            motortest.setPower(1);
            while(motortest.getCurrentPosition() < 1440 && opModeIsActive()) {

                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("heading:" ,formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("roll:" ,formatAngle(angles.angleUnit, angles.secondAngle));
                telemetry.addData("pitch:" ,formatAngle(angles.angleUnit, angles.thirdAngle));
                telemetry.addData("Position:", motortest.getCurrentPosition());
                telemetry.update();

            }
            motortest.setPower(0);
            telemetry.update();
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

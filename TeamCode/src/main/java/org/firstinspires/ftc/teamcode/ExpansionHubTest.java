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

    DcMotor motortest1;
    DcMotor motortest2;
    DcMotor motortest3;
    DcMotor motortest4;

    BNO055IMU imu;
    Orientation angles;
    Sensor sensorTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        motortest1 = hardwareMap.dcMotor.get("motor_test_1");
        motortest1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motortest1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motortest2 = hardwareMap.dcMotor.get("motor_test_2");
        motortest2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motortest2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motortest3 = hardwareMap.dcMotor.get("motor_test_3");
        motortest3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motortest3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motortest4 = hardwareMap.dcMotor.get("motor_test_4");
        motortest4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motortest4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sensorTelemetry = new Sensor(telemetry);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        sensorTelemetry.initSensor(imu);



        while (!opModeIsActive() && !Thread.currentThread().isInterrupted()) {

            telemetry.addLine("Start me please");
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading:" ,formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("roll:" ,formatAngle(angles.angleUnit, angles.secondAngle));
            telemetry.addData("pitch:" ,formatAngle(angles.angleUnit, angles.thirdAngle));
            telemetry.update();

        }

        while (opModeIsActive()) {
            /*motortest.setTargetPosition(1440);
            motortest.setPower(1);
            while(motortest.getCurrentPosition() < 1440 && opModeIsActive()) {

                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("heading:" ,formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("roll:" ,formatAngle(angles.angleUnit, angles.secondAngle));
                telemetry.addData("pitch:" ,formatAngle(angles.angleUnit, angles.thirdAngle));
                telemetry.addData("Position:", motortest.getCurrentPosition());
                telemetry.update();

            }
            motortest.setPower(0);*/
            runMotors(0.5);
            telemetry.update();
        }
    }

    void runMotors(double power) {

        motortest1.setPower(power);
        motortest2.setPower(power);
        motortest3.setPower(power);
        motortest4.setPower(power);

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

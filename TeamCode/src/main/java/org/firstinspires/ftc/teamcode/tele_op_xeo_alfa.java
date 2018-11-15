package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Xeo18-19: Tele-OP", group = "Xeo18-19")
public class tele_op_xeo_alfa extends OpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorRidicare;
    private DcMotor motorHex;
    private  DcMotor motorNebun;
    private double motor_power=0.66;
    private float currentPosition;
    private boolean coboara = false;
    private boolean urca = false;
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");
        motorHex = hardwareMap.dcMotor.get("motor_hex");
        motorNebun= hardwareMap.dcMotor.get("motor_nebun");
        motorRidicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRidicare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_power = 0.5;
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

      @Override
    public void loop(){

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading:" ,formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("roll:" ,formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("pitch:" ,formatAngle(angles.angleUnit, angles.thirdAngle));

        float gamepad1LeftY = -gamepad1.left_stick_y ;
        float gamepad1LeftX = -gamepad1.left_stick_x ;
        float gamepad1RightX = -gamepad1.right_stick_x;

        double angle = Math.toRadians(angles.firstAngle);

        double motorX = Math.cos(angle)* gamepad1LeftX - Math.sin(angle)*gamepad1LeftY;
        double motorY = Math.cos(angle)* gamepad1LeftY + Math.sin(angle)*gamepad1LeftX;

        gamepad1LeftX = (float)motorX;
        gamepad1LeftY = (float)motorY;

        // holonomic formulas
        float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        // clip the right/left values so that the values never exceed +/- 1
        FrontRight = Range.clip(FrontRight,  -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);
        if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left) {
            motorFrontRight.setPower(FrontRight * motor_power);
            motorFrontLeft.setPower(FrontLeft * motor_power);
            motorBackLeft.setPower(BackLeft * motor_power);
            motorBackRight.setPower(BackRight * motor_power);
        }
        /*if (gamepad1.a) {

            motorTest.setPower(1);

        }
        else {

            motorTest.setPower(0);

        }*/


        currentPosition = motorRidicare.getCurrentPosition();
        telemetry.addData("Encoder position", currentPosition);

        /*
        if(gamepad2.dpad_up){//coborare
            motorRidicare.setTargetPosition(950);
            motorRidicare.setPower(0.8);
        }
        if(currentPosition > 400){
            motorRidicare.setPower(0);
        }
        if(currentPosition < 200 && currentPosition > 150)
            motorRidicare.setPower(0);

        if(gamepad2.dpad_down){//ridicare
            motorRidicare.setTargetPosition(200);
            motorRidicare.setPower(-0.8);
        }*/
        //motorRidicare.setPower(gamepad2.left_stick_y*0.33f);

        motorNebun.setPower(-gamepad2.left_stick_y);

        if(gamepad2.dpad_up){//coborare
            motorRidicare.setPower(0.5);
            coboara = true;
        }
        if(coboara){
            if(currentPosition>400){
                motorRidicare.setPower(0);
            }
            if(currentPosition==800)  {
                motorRidicare.setPower(-0.05);
                dormi(100);}
            if(currentPosition>900){
                coboara = false;
            }
        }
        if(gamepad2.dpad_down){//urcare
            motorRidicare.setPower(-0.8);
            urca = true;
        }
        if(urca){
            if(currentPosition<300){
                motorRidicare.setPower(0);
                urca = false;
            }
        }

        motorHex.setPower(gamepad2.y? 1:0);


          telemetry.update();

      }
// + A - B
    @Override
    public void stop(){
        stopMotors();
        stopServos();
    }

    private void dormi(int x){
        long start_time = System.currentTimeMillis();
        while (System.currentTimeMillis() - start_time < x){
            int a;
        }
    }

    private void stopMotors() {

       /* motorBackRight.close();
        motorBackLeft.close();
        motorFrontRight.close();
        motorFrontLeft.close();*/
    }

    private void stopServos() {

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

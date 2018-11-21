package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
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
    private DcMotor motorLift;
    private DcMotor motorHex;
    private  DcMotor motorNebun;
    private Servo team;
    private double motor_power=0.66;
    private float currentPosition;
    private boolean coboara = false;
    private boolean urca = false;
    private BNO055IMU imu;
    private Orientation angles;
    private boolean urcaLift = false;
    private boolean coboaraLift = false;
    private final int maxExtindere = 500;
    private double ideal_speed=0.66;

    private static final String VUFORIA_KEY = "AaWc2RL/////AAABmZlxFvRyrk/YiqLbf3ykKmxLDRhJ5p955zNPuaCd9KvLm88Vfa399ERWzf+8iLlRqzO8q1Rl821vvtYMTJHhp6bE+zpOD8f5lcm6n14UM74JEVVwCDeogIBQSmNGzX4jkeCuK4VqC2rTZFlSB3DEY55XZfQ2vvcrjG1hfyls5tgUPhq5oI3XYehWhuoOaHYushaRmDLnCG5buNsJGHQFu7/XkHrEGTL5FMBFTKaaYKlxGJy7CVaIJrX794j2ispN9r9XaMgoNxXFHPIM6yGd6UFFwvMJ4YOXHWD3sdAWT+HLWmUxoyP+p29f7uchlma1y+1nGOO5pleCpk2at07Pp+E0pJifUgNg2Khhqa3SJABz";
    //private DigitalChannel butonFata;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");
        motorLift = hardwareMap.dcMotor.get("motor_lift");
        motorHex = hardwareMap.dcMotor.get("motor_hex");
        motorNebun= hardwareMap.dcMotor.get("motor_nebun");
        team=hardwareMap.servo.get("smart_servo");
        //butonFata = hardwareMap.digitalChannel.get("buton_fata");
        motorRidicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRidicare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorRidicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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


        currentPosition = motorRidicare.getCurrentPosition();
        telemetry.addData("Encoder position", currentPosition);

         if(gamepad1.a)
             goFront(ideal_speed);

          if(gamepad1.y)
              goFront(-ideal_speed);

          if(gamepad1.b)
              goRight(ideal_speed);

          if(gamepad1.x)
              goRight(-ideal_speed);

          if(!gamepad1.b && !gamepad1.a && !gamepad1.y && !gamepad1.x)
          {
              motorFrontLeft.setPower(0);
              motorFrontRight.setPower(0);
              motorBackRight.setPower(0);
              motorBackLeft. setPower(0);
          }

        if(gamepad2.dpad_down){//coborare
            motorRidicare.setPower(-0.5);
            coboara = true;
            urca = false;
        }
        if(coboara){
            if(currentPosition <= -750 && currentPosition >=- 800)  {
                motorRidicare.setPower(0.1);
                coboara = false;
                sleep(200);
                motorRidicare.setPower(0);
            }
            if(currentPosition<-500){
                motorRidicare.setPower(0);
                coboara = false;
            }
        }
        if(gamepad2.dpad_up)
        {//urcare
            motorRidicare.setPower(0.8);
            urca = true;
            coboara = false;
        }
        if(urca){
            if(currentPosition>-300){
                motorRidicare.setPower(0);
                urca = false;
            }
        }
if(gamepad2.b)   motorLift.setPower(0.96);

     if(gamepad2.a) motorLift.setPower(-0.96);

     if(!gamepad2.b && !gamepad2.a)
         motorLift.setPower(0);


        motorHex.setPower(gamepad2.y? 1:0);
        motorHex.setPower(gamepad2.x? -1:0);

        motorNebun.setPower(-gamepad2.left_stick_y);//intins brat

        telemetry.addData("position", motorLift.getCurrentPosition());
        telemetry.update();

      }
// + A - B
    @Override
    public void stop(){
        stopMotors();
        stopServos();
    }

    private void sleep(int x){
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

    private void goFront(double speed) {

        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft. setPower(-speed);

    }

    private void goRight(double speed) {

        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(-speed);
        motorBackRight.setPower(speed);
        motorBackLeft. setPower(speed);

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

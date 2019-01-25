package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovement;

@TeleOp(name = "Xeo18-19: Tele-OP-Timisoara", group = "Xeo18-19")
public class Teleop_Timisoara extends OpMode {
    private NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();
    // </movement>
    private DcMotor motorRidicare;
    private DcMotor motorLift;
    private DcMotor motorSurub;
    private Servo servo_team_mark;
    private boolean coboara = false;
    private boolean urca = false;
    private BNO055IMU imu;
    private Orientation angles;
    private CRServo servo_adunare_stanga;
    private CRServo servo_adunare_drepta;
    private Servo servo_cuva;

    private double vitezaMotoare = 0.66;

    private int brat_jos = -640;
    private int brat_sus = 0;
    private int treshold = 150;


    @Override
    public void init() {
        // <movement>
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setGamepad(gamepad1);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.TELEOP);
        robot.setMotorPower(vitezaMotoare);
        // </movement>

        servo_adunare_stanga = hardwareMap.crservo.get("servo_adunare_stanga");
        servo_adunare_drepta = hardwareMap.crservo.get("servo_adunare_drepta");
        servo_cuva = hardwareMap.servo.get("servo_cuva");

        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");
        motorLift = hardwareMap.dcMotor.get("motor_lift");
        motorSurub= hardwareMap.dcMotor.get("motor_nebun");
        servo_team_mark=hardwareMap.servo.get("smart_servo");

        motorRidicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRidicare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRidicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        */

        motorRidicare.setPower(-0.5);
        sleep(800);
        motorRidicare.setPower(0);
        motorRidicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRidicare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   }

   private int automatizare = 0;

    @Override
    public void loop() {
        robot.opModeLoop();

        if(gamepad2.x){
            motorSurub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSurub.setTargetPosition(0);
            motorSurub.setPower(0.5);
            servo_cuva.setPosition(0.35);
            automatizare = 1;
        }

        if(gamepad2.x && motorSurub.getCurrentPosition()<100){
            motorSurub.setPower(0);
            motorSurub.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRidicare.setPower(-0.9);
            servo_cuva.setPosition(0.35);// ???
            coboara = true;
            urca = false;
            automatizare = 2;
        }

        if(automatizare==2 && !urca){
            motorRidicare.setPower(0.9);
            urca = true;
            coboara = false;
            automatizare = 3;
        }
        if(automatizare==3 && !coboara){
            automatizare = 0;
        }


        controlViteza();
        controlPeri();
        controlBrat();
        controlSurub();
        controlLift();
        controlCuva();
    }



    @Override
    public void stop(){
        robot.stop();
    }

    private void controlViteza() {

        robot.setMotorPower(vitezaMotoare + gamepad1.right_trigger - gamepad1.left_trigger);

    }

    private void controlBrat(){
        int currentPosition = motorRidicare.getCurrentPosition();
        telemetry.addData("Motor brat: ", currentPosition);

        if(gamepad2.dpad_up){//urcare
            motorRidicare.setPower(-0.9);
            coboara = true;
            urca = false;
        }
        if(coboara){
            if(currentPosition<300){
                motorRidicare.setPower(0);
                coboara = false;
            }
        }
        if(gamepad2.dpad_down)
        {//coborare
            motorRidicare.setPower(0.9);
            urca = true;
            coboara = false;
        }
        if(urca){
            if(currentPosition<500){
                motorRidicare.setPower(0);
                urca = false;
            }
        }
    }

    private void controlSurub(){
        if(automatizare==0)
            motorSurub.setPower(gamepad2.left_stick_y);
    }

    private void controlLift(){
        double lift_power = gamepad2.right_stick_y;
        motorLift.setPower(lift_power);
    }

    private void controlPeri(){
        if (gamepad2.a){
            servo_adunare_drepta.setPower(1);
            servo_adunare_stanga.setPower(-1);
            return;
        }
        if (gamepad2.b) {
            servo_adunare_drepta.setPower(-1);
            servo_adunare_stanga.setPower(1);
            return;
        }
        servo_adunare_drepta.setPower(0);
        servo_adunare_stanga.setPower(0);
    }

    boolean sus = false;
    private void controlCuva() {

        if (gamepad2.right_bumper)
            servo_cuva.setPosition(0);

        if (gamepad2.left_bumper)
            servo_cuva.setPosition(0.35);

    }

    private void sleep(int x){
        long start_time = System.currentTimeMillis();
        while (System.currentTimeMillis() - start_time < x){
            int a;
        }
    }
}

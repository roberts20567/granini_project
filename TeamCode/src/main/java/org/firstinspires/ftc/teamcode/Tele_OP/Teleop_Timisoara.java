package org.firstinspires.ftc.teamcode.Tele_OP;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovement;

@TeleOp(name = "TeleOP Oradea")
public class Teleop_Timisoara extends OpMode {
    private NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();
    // </movement>
    private DcMotor motorRidicare;
    private DcMotor motorLift;
    private DcMotor motorLift2;
    private Lift lift;

    private DcMotor motorSurub;
    private Servo servo_team_mark;
    private boolean coboara = false;
    private boolean urca = false;
    private BNO055IMU imu;
    private Orientation angles;
    private CRServo servo_adunare_stanga;
    private CRServo servo_adunare_drepta;
    private Servo servo_cuva;

    private double vitezaMotoare = 0.44;

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
        motorLift2 = hardwareMap.dcMotor.get("frate_motor_lift");
        lift = new Lift(motorLift, motorLift2);

        motorSurub= hardwareMap.dcMotor.get("motor_nebun");
        motorSurub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSurub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        //motorRidicare.setPower(-0.5);
        //sleep(800);
        motorRidicare.setPower(0);
        motorRidicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRidicare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   }

    @Override
    public void loop() {
        robot.opModeLoop();
        automatizare();
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

    private int auto = 0;
    private void automatizare(){
        telemetry.addData("extindere", motorSurub.getCurrentPosition());

        if(gamepad2.x && auto==0){
            auto = 1;
            motorSurub.setPower(-0.9);
            servo_cuva.setPosition(0);
        }else if(auto==1 && motorSurub.getCurrentPosition()>-0){
            auto = 0;
            servo_cuva.setPosition(0.35);
            sleep(400);
            motorRidicare.setPower(-0.9);
            sleep(600);
            motorRidicare.setPower(0);
        }

        if(motorSurub.getCurrentPosition()<0 && auto==0){
            servo_cuva.setPosition(0.05);
        }
    }

    private void controlViteza() {
        robot.setMotorPower(vitezaMotoare * (gamepad1.right_trigger + 1) / (gamepad1.left_trigger + 1));

    }

    private void controlBrat(){
        int currentPosition = motorRidicare.getCurrentPosition();
        telemetry.addData("Motor brat: ", currentPosition);
        if(gamepad2.dpad_up){
            motorRidicare.setPower(-0.9);
            sleep(600);
            motorRidicare.setPower(0);
        }
        if(gamepad2.dpad_down){
            motorRidicare.setPower(0.5);
            sleep(300);
            motorRidicare.setPower(0);
        }
    }

    private void controlSurub(){
        if(auto==0) {
            motorSurub.setPower(-gamepad2.left_stick_y);
        }
    }

    private void controlLift(){
        double lift_power = gamepad2.right_stick_y;
        lift.setPower(lift_power);
        telemetry.addData("lift", motorLift.getCurrentPosition());

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

        if(gamepad2.right_bumper)
            servo_cuva.setPosition(0);

        if(gamepad2.left_bumper)
            servo_cuva.setPosition(0.35);

    }

    private void sleep(int x){
        long start_time = System.currentTimeMillis();
        while (System.currentTimeMillis() - start_time < x){
            int a;
        }
    }
}

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
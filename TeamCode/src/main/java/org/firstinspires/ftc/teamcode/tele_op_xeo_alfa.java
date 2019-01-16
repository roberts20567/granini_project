package org.firstinspires.ftc.teamcode;

        import android.widget.Button;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
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

        import static com.sun.tools.doclint.Entity.and;
        import static com.sun.tools.doclint.Entity.or;

@TeleOp(name = "Xeo18-19: Tele-OP", group = "Xeo18-19")
public class tele_op_xeo_alfa extends OpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorRidicare;
    private DcMotor motorLift;
    private DcMotor motorHex;
    private DcMotor motorSurub;
    private DigitalChannel digitalTouch;
    private Servo team;
    private float currentPosition;
    private boolean coboara = false;
    private boolean urca = false;
    private BNO055IMU imu;
    private Orientation angles;
    private boolean urcaLift = false;
    private boolean coboaraLift = false;
    private final int maxExtindere = 500;
    private double ideal_speed=0.66;

    private CRServo servo_adunare_stanga;
    private CRServo servo_adunare_drepta;

    private static final String VUFORIA_KEY = "AaWc2RL/////AAABmZlxFvRyrk/YiqLbf3ykKmxLDRhJ5p955zNPuaCd9KvLm88Vfa399ERWzf+8iLlRqzO8q1Rl821vvtYMTJHhp6bE+zpOD8f5lcm6n14UM74JEVVwCDeogIBQSmNGzX4jkeCuK4VqC2rTZFlSB3DEY55XZfQ2vvcrjG1hfyls5tgUPhq5oI3XYehWhuoOaHYushaRmDLnCG5buNsJGHQFu7/XkHrEGTL5FMBFTKaaYKlxGJy7CVaIJrX794j2ispN9r9XaMgoNxXFHPIM6yGd6UFFwvMJ4YOXHWD3sdAWT+HLWmUxoyP+p29f7uchlma1y+1nGOO5pleCpk2at07Pp+E0pJifUgNg2Khhqa3SJABz";

    // variabile Claudiu
    private final double VITEZA_MISCARE_1 = 0.22;
    private final double VITEZA_MISCARE_2 = 0.66;
    private final double VITEZA_MISCARE_3 = 0.99;
    private boolean miscare_POW = false;
    private double start_angle = 0;
    private double motor_power = VITEZA_MISCARE_2;

    private double POZITIE_LIFT_MAXIM = 2150;
    private double POZITIE_LIFT_MINIM = 100;
    private double DISTANTA_INCETINIRE_LIFT = 400;

    private int automatizare;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");

        //motorLift = hardwareMap.dcMotor.get("motor_lift");
        //motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motorHex = hardwareMap.dcMotor.get("motor_hex");
        motorSurub= hardwareMap.dcMotor.get("motor_nebun");
        team=hardwareMap.servo.get("smart_servo");
        //butonFata = hardwareMap.digitalChannel.get("buton_fata");

       /* motorSurub.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSurub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

     //  motorRidicare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*motorRidicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRidicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        motor_power = 0.5;

        automatizare = 0;

        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /*NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();
        robot.attachMotors(motorFrontRight, motorBackLeft, motorBackRight, motorBackLeft);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.TELEOP);
        robot.setMotorPower(0.66);
        robot.opModeLoop();
*/
        servo_adunare_stanga = hardwareMap.crservo.get("servo_adunare_stanga");
        servo_adunare_drepta = hardwareMap.crservo.get("servo_adunare_drepta");
    }

    private void Miscare(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading:" ,formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("roll:" ,formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("pitch:" ,formatAngle(angles.angleUnit, angles.thirdAngle));

        if(gamepad1.y)
            motor_power = VITEZA_MISCARE_1;
        if(gamepad1.a)
            motor_power = VITEZA_MISCARE_2;
        if(gamepad1.x)
            motor_power = VITEZA_MISCARE_3;

        double sprint = gamepad1.right_trigger;

        float gamepad1LeftY = -gamepad1.left_stick_y ;
        float gamepad1LeftX = -gamepad1.left_stick_x ;
        float gamepad1RightX = -gamepad1.right_stick_x;

        if(miscare_POW) {
            double angle = Math.toRadians(angles.firstAngle - start_angle);
            double motorX = Math.cos(angle) * gamepad1LeftX - Math.sin(angle) * gamepad1LeftY;
            double motorY = Math.cos(angle) * gamepad1LeftY + Math.sin(angle) * gamepad1LeftX;
            gamepad1LeftX = (float) motorX;
            gamepad1LeftY = (float) motorY;
        }

        if (gamepad1.left_bumper){
            miscare_POW = false;
        }
        if (gamepad1.right_bumper){
            miscare_POW = true;
            start_angle = angles.firstAngle;
        }

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
            motorFrontRight.setPower(FrontRight * (motor_power +sprint));
            motorFrontLeft.setPower(FrontLeft * (motor_power + sprint));
            motorBackLeft.setPower(BackLeft * (motor_power + sprint));
            motorBackRight.setPower(BackRight * (motor_power + sprint));
        }
    }

    private void controlBrat(){
        currentPosition = motorRidicare.getCurrentPosition();
        telemetry.addData("Motor brat: ", currentPosition);

        if(gamepad2.dpad_down){//coborare
            motorRidicare.setPower(-0.8);
            coboara = true;
            urca = false;
        }
        if(coboara){
            if(currentPosition <= -750 && currentPosition >= -800)  {
                motorRidicare.setPower(0.1);
                coboara = false;
                sleep(300);
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
    }

    private void controlSurub(){
        //telemetry.addData("encoder surub", motorSurub.getCurrentPosition());
        double position = motorSurub.getCurrentPosition();

            motorSurub.setPower(-gamepad2.left_stick_y);

        if (gamepad2.x)
        {
            motorSurub.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSurub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorSurub.setTargetPosition(10);
            motorSurub.setPower(0.66);
            automatizare = 1;
        }
        if (automatizare == 1 && !motorSurub.isBusy()){
            automatizare = 2;
            urca = true;
        }
        if (automatizare == 2 && !urca){
            automatizare = 0;
            coboara = true;
        }
        //TODO automatizare lift
    }

    private void controlLift(){
        //double lift_position = -motorLift.getCurrentPosition();
        //double lift_power = gamepad2.right_stick_y;
/*
        telemetry.addData("Lift", lift_position);
        telemetry.addData("Lift", lift_power);

        if(lift_power < 0){
            if (lift_position < POZITIE_LIFT_MAXIM - DISTANTA_INCETINIRE_LIFT)
                motorLift.setPower(lift_power);
            else if(lift_position < POZITIE_LIFT_MAXIM)
                motorLift.setPower(lift_power * ((POZITIE_LIFT_MAXIM-lift_position) / DISTANTA_INCETINIRE_LIFT));
            else
                motorLift.setPower(0);
            return;
        }

        if(lift_power > 0){
            if (lift_position > POZITIE_LIFT_MINIM + DISTANTA_INCETINIRE_LIFT)
                motorLift.setPower(lift_power);
            else if(lift_position > POZITIE_LIFT_MINIM)
                motorLift.setPower(lift_power * ((lift_position - POZITIE_LIFT_MINIM) / DISTANTA_INCETINIRE_LIFT));
            else
                motorLift.setPower(0);
            return;
        }*/

       //motorLift.setPower(lift_power);
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

    @Override
    public void loop(){

        Miscare();

        controlBrat();

        controlSurub();

        controlPeri();

        controlLift();

        //motorSurub.setPower(-gamepad2.left_stick_y);

        if (gamepad2.dpad_left) {

            motorRidicare.setPower(-1);

        }

        else if (gamepad2.dpad_right) {

            motorRidicare.setPower(1);

        }

        else
            motorRidicare.setPower(0);

        double position = motorSurub.getCurrentPosition();
        telemetry.addData("extindere ", position);
        telemetry.addData("automatizare ", automatizare);
        //telemetry.addData("Button state: ", digitalTouch.getState());
        telemetry.update();
    }
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

    private void stopServos() {

    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

   private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

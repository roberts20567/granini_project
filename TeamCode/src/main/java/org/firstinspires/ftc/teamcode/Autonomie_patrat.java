package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "Autonomie_patrat")//duck duck duck duck duck duck duck duck duck duck duck duck duck duck duck
public class Autonomie_patrat extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorSurub;
    private DcMotor motorHex;
    private DcMotor motorRidicare;
    private DcMotor motorLift;
    private Servo servoTeamMarker;
    private double motor_power=0.66;
    private float currentPosition;
    private boolean coboara = false;
    private boolean urca = false;
    private BNO055IMU imu;
    private Orientation angles;

    private void initGyro(){
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

    private void Init(){
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        motorLift = hardwareMap.dcMotor.get("motor_lift");
        motorHex = hardwareMap.dcMotor.get("motor_hex");
        motorSurub = hardwareMap.dcMotor.get("motor_nebun");
        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");
        servoTeamMarker =hardwareMap.servo.get("smart_servo");


    }

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        initGyro();
        while (!isStarted()){
            telemetry.addData("Back Left: ", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right: ", motorBackRight.getCurrentPosition());
            telemetry.addData("Front Left: ", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Front Right: ", motorFrontRight.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();

        motorRidicare.setPower(-0.8);
        Thread.sleep(500);
        motorRidicare.setPower(0);
        Thread.sleep(1000);

        motorLift.setPower(-0.4);
        Thread.sleep(999);
        motorLift.setPower(0);
        Thread.sleep(1000);

        mergiDreapta(0.3);
        Thread.sleep(100);
        oprireMiscare();
        Thread.sleep(1000);

        motorRidicare.setPower(0.8);
        Thread.sleep(900);
        motorRidicare.setPower(0);
        Thread.sleep(1000);

        rotireStanga(0.3);
        Thread.sleep(100);
        oprireMiscare();
        Thread.sleep(1000);

        mergiFata(0.66);
        Thread.sleep(2100);
        oprireMiscare();

        rotireGoogle(40, 0.4);
        servoTeamMarker.setPosition(-1);
        Thread.sleep(1000);
        servoTeamMarker.setPosition(1);
        Thread.sleep(1000);
        rotireGoogle(250, 0.4);



        Thread.sleep(500);
        mergiSpate(1);
        Thread.sleep(2250);
        oprireMiscare();



    }

    private void mergiFata(double speed) {
        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft. setPower(-speed);
    }

    private void mergiSpate(double viteza){
        mergiFata(-viteza);
    }

    private void mergiDreapta(double viteza) {

        motorFrontLeft.setPower(viteza);
        motorFrontRight.setPower(viteza);
        motorBackRight.setPower(-viteza);
        motorBackLeft. setPower(-viteza);

    }

    private void mergiStanga(double viteza){
        mergiDreapta(-viteza);
    }

    private void rotireDreapta(double viteza) {
        motorFrontLeft.setPower(viteza);
        motorBackRight.setPower(viteza);
        motorFrontRight.setPower(viteza);
        motorBackLeft.setPower(viteza);
    }

    private void rotireStanga(double viteza) {
        rotireDreapta(-viteza);
    }

    private void oprireMiscare(){
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    /*private void rotireLaUnghiGyro(int unghi)
    {
        int opus = Math.abs(unghi-180);
        double alpha = 0;
        double unghi_initial = angles.firstAngle;
        while (angles.firstAngle != unghi && opModeIsActive())
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            alpha = Math.abs(unghi-(angles.firstAngle));
            if (unghi < opus)
                rotireStanga(Math.sqrt(alpha/unghi_initial));

            if (unghi >= opus)
                rotireDreapta(Math.sqrt(alpha/unghi_initial));

        }
        oprireMiscare();
    }

    private void rotireLaUnghiEncodere(int unghi)
    {

    }*/

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    private void rotireGoogle(double degrees, double power) {
        resetAngle();
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees < 0) {
            // turn right.
            rotireDreapta(power);
        }
        else if (degrees > 0) {
            // turn left.
            rotireStanga(power);
        }
        else return;
        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {
                double alpha = Math.abs(degrees-getAngle()) / degrees;
                rotireDreapta(0.9*power * Math.sqrt(alpha) + 0.1);
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                double alpha = Math.abs(degrees-getAngle()) / degrees;
                rotireStanga(0.9*power * Math.sqrt(alpha) + 0.1);
            }
        // turn the motors off.
        oprireMiscare();
        // reset angle tracking on new heading.
        resetAngle();
    }

    private double min(double a, double b){
        return a<b ? a : b;
    }

    private double globalAngle;
    private Orientation lastAngles = new Orientation();

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}

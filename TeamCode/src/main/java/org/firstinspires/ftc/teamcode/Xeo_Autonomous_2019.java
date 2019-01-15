package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.OmniDirectionalMovement;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "Xeo_Autonomous_2019")
public class Xeo_Autonomous_2019 extends LinearOpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private DcMotor motorRidicare;
    private DcMotor motorLift;
    private DcMotor motorHex;
    private  DcMotor motorNebun;
    private double motor_power=0.66;
    private float currentPosition;
    private boolean coboara = false;
    private boolean urca = false;
    private int pos = 0;
    BNO055IMU imu;
    Orientation angles;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AaWc2RL/////AAABmZlxFvRyrk/YiqLbf3ykKmxLDRhJ5p955zNPuaCd9KvLm88Vfa399ERWzf+8iLlRqzO8q1Rl821vvtYMTJHhp6bE+zpOD8f5lcm6n14UM74JEVVwCDeogIBQSmNGzX4jkeCuK4VqC2rTZFlSB3DEY55XZfQ2vvcrjG1hfyls5tgUPhq5oI3XYehWhuoOaHYushaRmDLnCG5buNsJGHQFu7/XkHrEGTL5FMBFTKaaYKlxGJy7CVaIJrX794j2ispN9r9XaMgoNxXFHPIM6yGd6UFFwvMJ4YOXHWD3sdAWT+HLWmUxoyP+p29f7uchlma1y+1nGOO5pleCpk2at07Pp+E0pJifUgNg2Khhqa3SJABz";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initIMU();
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");
        motorHex = hardwareMap.dcMotor.get("motor_hex");

        motorLift = hardwareMap.dcMotor.get("motor_lift");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else {
                                    silverMineral1X = (int) recognition.getLeft();
                                }
                            }

                            coborareLift();
                            rotireGoogle(0, 0.5);
                            Thread.sleep(100);
                            if (goldMineralX == -1) {
                                goRight(0.55+0.11);
                                sleep(400);
                                oprireMiscare();
                                Thread.sleep(100);

                                telemetry.addData("Gold Mineral Position", "Left");
                            }else if(goldMineralX < silverMineral1X) {
                                iaCub(2);
                                telemetry.addData("Gold Mineral Position", "Center");
                            }else {
                                telemetry.addData("Gold Mineral Position", "Right");
                            }
                        }
                    }
                        telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initIMU() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    private void iaCub(int pozitie) throws InterruptedException {

        switch (pozitie) {

            case 1:
                break;
            case 2:
                motorRidicare.setPower(-0.8);
                Thread.sleep(500);
                motorRidicare.setPower(0);
                Thread.sleep(100);
                goFront(0.7);
                motorHex.setPower(0.66);
                Thread.sleep(800);
                oprireMiscare();
                motorHex.setPower(0);
                Thread.sleep(100);
                break;
            case 3:
                break;

        }

    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void coborareLift() throws InterruptedException {

        motorRidicare.setPower(-0.8);
        Thread.sleep(500);
        motorRidicare.setPower(0);
        Thread.sleep(100);
        motorRidicare.setPower(0.8);
        Thread.sleep(900);
        motorRidicare.setPower(0);
        Thread.sleep(100);
        motorLift.setPower(-0.4);
        Thread.sleep(1500);
        motorLift.setPower(0);
        goRight(-0.3);
        Thread.sleep(281);
        oprireMiscare();

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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
    private void goBack(double viteza){
        goFront(-viteza);
    }

    private void rotireDreapta(double viteza)
    {
        motorFrontLeft.setPower(viteza);
        motorBackRight.setPower(viteza);
        motorFrontRight.setPower(viteza);
        motorBackLeft.setPower(viteza);
    }

    private void oprireMiscare(){
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
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

    private void rotireStanga(double viteza)
    {
        rotireDreapta(-viteza);
    }

    private void mergiFata(float viteza)
    {
        motorFrontRight.setPower(-viteza);
    }


}

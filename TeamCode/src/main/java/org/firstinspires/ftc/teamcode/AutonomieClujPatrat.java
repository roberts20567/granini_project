package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiu;
import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovement;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

@Autonomous(name="Autonomie Vuforia Cluj")
public class AutonomieClujPatrat extends LinearOpMode {

    private NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();
    private DcMotor motorLift;
    private Servo servo_team_mark;
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorRidicare;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AaWc2RL/////AAABmZlxFvRyrk/YiqLbf3ykKmxLDRhJ5p955zNPuaCd9KvLm88Vfa399ERWzf+8iLlRqzO8q1Rl821vvtYMTJHhp6bE+zpOD8f5lcm6n14UM74JEVVwCDeogIBQSmNGzX4jkeCuK4VqC2rTZFlSB3DEY55XZfQ2vvcrjG1hfyls5tgUPhq5oI3XYehWhuoOaHYushaRmDLnCG5buNsJGHQFu7/XkHrEGTL5FMBFTKaaYKlxGJy7CVaIJrX794j2ispN9r9XaMgoNxXFHPIM6yGd6UFFwvMJ4YOXHWD3sdAWT+HLWmUxoyP+p29f7uchlma1y+1nGOO5pleCpk2at07Pp+E0pJifUgNg2Khhqa3SJABz";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private int target = 1267;

    @Override
    public void runOpMode() throws InterruptedException {
        // <movement>
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        motorLift = hardwareMap.dcMotor.get("motor_lift");
        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");
        servo_team_mark=hardwareMap.servo.get("smart_servo");

        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.AUTONOMOUS);
        robot.setMotorPower(0.44);
        // </movement>

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        motorLift.setPower(0.11);

        waitForStart();
        telemetry.addData("Position BL:" ,motorBackLeft.getCurrentPosition());
        telemetry.addData("Position BR:" ,motorBackRight.getCurrentPosition());
        telemetry.addData("Position FL:" ,motorFrontLeft.getCurrentPosition());
        telemetry.addData("Position FR:" ,motorFrontRight.getCurrentPosition());
        telemetry.update();

        if (tfod != null) {
            tfod.activate();
        }

        motorLift.setPower(0);
        coborareRobot();
        sleep(2000);
        robot.moveToDirection(400, 90);
        sleep(1500);

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

                    Thread.sleep(100);
                    if (goldMineralX == -1) {

                        telemetry.addData("Gold Mineral Position", "Left");
                    }else if(goldMineralX < silverMineral1X) {

                        telemetry.addData("Gold Mineral Position", "Center");
                    }else {

                        telemetry.addData("Gold Mineral Position", "Right");
                    }
                }
            }
            telemetry.update();
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        //resetEncoders();
        robot.moveToDirection(2850, 0);
        sleep(3000);
        //resetEncoders();
        robot.rotateToAngle(-165);
        sleep(3000);
        robot.moveToDirection(1000, 90);
        sleep(3000);

        servo_team_mark.setPosition(-1);
        sleep(2000);

        robot.rotateToAngle(110);
        sleep(2500);
        robot.moveToDirection(4000, -90);
        sleep(5000);
        motorRidicare.setPower(0.5);
        sleep(2000);
        motorRidicare.setPower(0);
        sleep(11);
    }

    private void coborareRobot() {

        motorLift.setPower(-0.5);
        sleep(2000);
        motorLift.setPower(-0.11);
        motorRidicare.setPower(-0.75);
        sleep(1000);
        motorRidicare.setPower(0);

    }

    private void resetEncoder(DcMotor motor) {

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void resetEncoders() {

        resetEncoder(motorBackLeft);
        resetEncoder(motorBackRight);
        resetEncoder(motorFrontLeft);
        resetEncoder(motorFrontRight);

    }

}

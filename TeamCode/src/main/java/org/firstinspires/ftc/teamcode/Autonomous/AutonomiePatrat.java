package org.firstinspires.ftc.teamcode.Autonomous;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovement;

import java.util.List;

@Autonomous(name = "Patrat :)")
public class AutonomiePatrat extends LinearOpMode {
    private NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private DcMotor motorLift;
    private DcMotor frateMotorLift;
    private Lift lift;

    private Servo servo_team_mark;
    private DcMotor motorFrontRight;
    private DcMotor motorRidicare;
    private MediaPlayer mPlayer;

    private CRServo servo_adunare_stanga;
    private CRServo servo_adunare_drepta;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AaWc2RL/////AAABmZlxFvRyrk/YiqLbf3ykKmxLDRhJ5p955zNPuaCd9KvLm88Vfa399ERWzf+8iLlRqzO8q1Rl821vvtYMTJHhp6bE+zpOD8f5lcm6n14UM74JEVVwCDeogIBQSmNGzX4jkeCuK4VqC2rTZFlSB3DEY55XZfQ2vvcrjG1hfyls5tgUPhq5oI3XYehWhuoOaHYushaRmDLnCG5buNsJGHQFu7/XkHrEGTL5FMBFTKaaYKlxGJy7CVaIJrX794j2ispN9r9XaMgoNxXFHPIM6yGd6UFFwvMJ4YOXHWD3sdAWT+HLWmUxoyP+p29f7uchlma1y+1nGOO5pleCpk2at07Pp+E0pJifUgNg2Khhqa3SJABz";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private void initRobot(){
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");

        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.AUTONOMOUS);
        robot.setMotorPower(0.5);

        motorLift = hardwareMap.dcMotor.get("motor_lift");
        frateMotorLift = hardwareMap.dcMotor.get("frate_motor_lift");
        lift = new Lift(motorLift, frateMotorLift);

        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");
        servo_team_mark=hardwareMap.servo.get("smart_servo");

        servo_adunare_stanga = hardwareMap.crservo.get("servo_adunare_stanga");
        servo_adunare_drepta = hardwareMap.crservo.get("servo_adunare_drepta");
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void initSound() {
        mPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.mariothemesong1);
    }

    private int detectieGold() {
        int gold_mineral_position = 0;
        if (tfod != null) {
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
                    if (goldMineralX == -1) {
                        gold_mineral_position = 1;
                    }else if(goldMineralX < silverMineral1X) {
                        gold_mineral_position = 2;
                    }else {
                        gold_mineral_position = 3;
                    }
                }
            }
            tfod.shutdown();
        }
        return gold_mineral_position;
    }

    private void doboaraGoldDreapta(){
        robot.rotateToAngle(25);
        sleep(1200);

        robot.moveToDirectionCentimeters(105, 0);
        sleep(4000);

        robot.moveToDirectionCentimeters(100, 90);
        sleep(3000);

        servo_team_mark.setPosition(-1);
        sleep(500);

        robot.rotateToAngle(110);
        sleep(2500);

        robot.moveToDirectionCentimeters(180, -90);
        sleep(5000);

        motorRidicare.setPower(0.5);
        sleep(1500);
        motorRidicare.setPower(0);

        sleep(11);
    }

    private void doboaraGoldMijloc(){
        robot.moveToDirection(1000, 0);
        sleep(2000);
        robot.moveToDirection(600, -90);
        sleep(2000);
        robot.moveToDirection(1000, 0);
        sleep(2000);
    }

    private void doboaraGoldStanga(){
        robot.rotateToAngle(-35);
        sleep(1200);
        robot.moveToDirection(2500, 0);
        sleep(4000);
        robot.rotateToAngle(-100);
        sleep(2000);
        robot.moveToDirectionCentimeters(50, 180);
        sleep(2000);
        servo_team_mark.setPosition(-1);
        robot.moveToDirectionCentimeters(150, 0);
        sleep(5000);
        robot.rotateToAngle(90);
        sleep(1500);
        motorRidicare.setPower(0.5);
        sleep(1000);
        motorRidicare.setPower(0);
    }

    private void doboaraNimic(){
        robot.moveToDirection(2850, 0);
        sleep(3000);
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
        lift.setPower(-0.5);
        sleep(2000);
        lift.setPower(-0.11);
        motorRidicare.setPower(-0.75);
        sleep(1000);
        motorRidicare.setPower(0);
        sleep(2000);
        robot.moveToDirection(400, 90);
        sleep(1500);
    }

    private void iaCubStanga(){
        robot.moveToDirectionCentimeters(10, 0);
        sleep(1000);

        robot.rotateToAngle(70);
        sleep(1000);

        motorRidicare.setPower(0.5);
        sleep(500);

        motorRidicare.setPower(0);
        servo_adunare_drepta.setPower(1);
        servo_adunare_stanga.setPower(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(70, -90);
        sleep(2500);

        servo_adunare_drepta.setPower(0);
        servo_adunare_stanga.setPower(0);
        motorRidicare.setPower(-1);
        sleep(600);
        motorRidicare.setPower(-0.11);

        robot.moveToDirectionCentimeters(40, -90);
        sleep(1000);

        robot.rotateToAngle(-106);
        sleep(1700);

        robot.moveToDirectionCentimeters(40, 90);
        sleep(1500);

        servo_team_mark.setPosition(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(170, -90);
        sleep(3000);
        servo_team_mark.setPosition(0);

        motorRidicare.setPower(0.5);
        sleep(500);
        motorRidicare.setPower(0);

    }

    private void iaCubMijloc(){
        robot.moveToDirectionCentimeters(10, 0);
        sleep(1000);

        robot.rotateToAngle(100);
        sleep(1000);

        motorRidicare.setPower(0.5);
        sleep(500);
        motorRidicare.setPower(0);

        servo_adunare_drepta.setPower(1);
        servo_adunare_stanga.setPower(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(70, -90);
        sleep(1800);

        servo_adunare_drepta.setPower(0);
        servo_adunare_stanga.setPower(0);
        motorRidicare.setPower(-1);
        sleep(600);
        motorRidicare.setPower(-0.11);

        robot.moveToDirectionCentimeters(40, -90);
        sleep(1000);

        robot.rotateToAngle(-140);
        sleep(1800);

        robot.moveToDirectionCentimeters(30, 0);
        sleep(1000);

        servo_team_mark.setPosition(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(170,-90);
        sleep(3000);

        motorRidicare.setPower(0.5);
        sleep(500);
        motorRidicare.setPower(0);
    }

    private void iaCubDreapta(){
        robot.moveToDirectionCentimeters(10, 0);
        sleep(1000);

        robot.rotateToAngle(130);
        sleep(1000);

        motorRidicare.setPower(0.5);
        sleep(500);
        motorRidicare.setPower(0);

        servo_adunare_drepta.setPower(1);
        servo_adunare_stanga.setPower(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(70, -90);
        sleep(2000);

        servo_adunare_drepta.setPower(0);
        servo_adunare_stanga.setPower(0);
        motorRidicare.setPower(-1);
        sleep(600);
        motorRidicare.setPower(-0.11);

        robot.moveToDirectionCentimeters(40, -90);
        sleep(1000);

        robot.rotateToAngle(-170);
        sleep(2000);

        robot.moveToDirectionCentimeters(70,0);
        sleep(2000);

        servo_team_mark.setPosition(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(185, -90);
        sleep(2800);

        motorRidicare.setPower(0.5);
        sleep(500);
        motorRidicare.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initVuforia();
        initTfod();
        initSound();
        //lift.setPower(0.1);
        if(tfod != null){
            tfod.activate();
            sleep(500);
        }
        waitForStart();

        mPlayer.start();

        int gold_position = detectieGold();
        //coborareRobot();
        telemetry.addData("Gold position", gold_position);
        telemetry.update();

        switch (gold_position){
            case 0:
                //doboaraNimic();
            case 1:
                iaCubStanga();
                break;
            case 2:
                iaCubMijloc();
                break;
            case 3:
                iaCubDreapta();
                break;
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

package org.firstinspires.ftc.teamcode.Autonomous;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name = "Crater :)")
public class AutonomieCrater extends LinearOpMode {
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

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AaWc2RL/////AAABmZlxFvRyrk/YiqLbf3ykKmxLDRhJ5p955zNPuaCd9KvLm88Vfa399ERWzf+8iLlRqzO8q1Rl821vvtYMTJHhp6bE+zpOD8f5lcm6n14UM74JEVVwCDeogIBQSmNGzX4jkeCuK4VqC2rTZFlSB3DEY55XZfQ2vvcrjG1hfyls5tgUPhq5oI3XYehWhuoOaHYushaRmDLnCG5buNsJGHQFu7/XkHrEGTL5FMBFTKaaYKlxGJy7CVaIJrX794j2ispN9r9XaMgoNxXFHPIM6yGd6UFFwvMJ4YOXHWD3sdAWT+HLWmUxoyP+p29f7uchlma1y+1nGOO5pleCpk2at07Pp+E0pJifUgNg2Khhqa3SJABz";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    void initRobot(){
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");

        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.AUTONOMOUS);
        robot.setMotorPower(0.4);

        motorLift = hardwareMap.dcMotor.get("motor_lift");
        frateMotorLift = hardwareMap.dcMotor.get("frate_motor_lift");
        lift = new Lift(motorLift, frateMotorLift);

        motorRidicare = hardwareMap.dcMotor.get("motor_ridicare");
        servo_team_mark=hardwareMap.servo.get("smart_servo");
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
                        telemetry.addData("Gold Mineral Position", "Left");
                    }else if(goldMineralX < silverMineral1X) {
                        gold_mineral_position = 2;
                        telemetry.addData("Gold Mineral Position", "Center");
                    }else {
                        gold_mineral_position = 3;
                        telemetry.addData("Gold Mineral Position", "Right");
                    }
                }
            }
            telemetry.update();
            tfod.shutdown();
        }
        return gold_mineral_position;
    }

    private void doboaraGoldDreapta(){
        robot.rotateToAngle(30);
        sleep(1200);
        robot.moveToDirection(2000, 0);
        sleep(3000);
    }

    private void doboaraGoldMijloc(){
        robot.moveToDirection(1000, 0);
        sleep(2000);
        robot.moveToDirection(500, -90);
        sleep(2000);
        robot.moveToDirection(1000, 0);
        sleep(2000);
    }

    private void doboaraGoldStanga(){
        robot.rotateToAngle(-35);
        sleep(1200);
        robot.moveToDirection(2500, 0);
        sleep(5000);
    }

    private void doboaraNimic(){
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

    private void dezgatareRobot() {
        lift.setPower(-0.5);
        sleep(1700);
        lift.setPower(0);
        motorRidicare.setPower(-0.75);
        sleep(1000);
        motorRidicare.setPower(-0.11);
        robot.moveToDirection(500, 90);
        sleep(2000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initVuforia();
        initTfod();
        initSound();
        lift.setPower(0.1);
        if(tfod != null){
            tfod.activate();
        }

        waitForStart();
        mPlayer.start();
        sleep(1000);
        int gold_position = detectieGold();
        for (int i=0; i<7 && gold_position==0; i++) {
            telemetry.addData("Gold po1", gold_position);
            telemetry.update();
            gold_position = detectieGold();
            sleep(500);
        }
        sleep(1000);

        dezgatareRobot();
        sleep(2000);
        telemetry.addData("Gold position", gold_position);
        telemetry.update();

        switch (gold_position){
            case 0:
                doboaraNimic();
            case 1:
                doboaraGoldStanga();
                break;
            case 2:
                doboaraGoldMijloc();
                break;
            case 3:
                doboaraGoldDreapta();
                break;
        }
        sleep(9999);

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

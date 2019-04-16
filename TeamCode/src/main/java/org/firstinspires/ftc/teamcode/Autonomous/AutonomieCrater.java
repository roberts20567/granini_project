package org.firstinspires.ftc.teamcode.Autonomous;

import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private DcMotor motorFrontRight;

    ModernRoboticsI2cRangeSensor rangeSensor;

    private DcMotor motorArticulatie;
    private DcMotor motorLift;
    private DcMotor motorTijaFiletata;
    private DcMotor motorExtindere;

    private CRServo servo_adunare_stanga;
    private CRServo servo_adunare_dreapta;
    private Servo servo_cuva;


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AaWc2RL/////AAABmZlxFvRyrk/YiqLbf3ykKmxLDRhJ5p955zNPuaCd9KvLm88Vfa399ERWzf+8iLlRqzO8q1Rl821vvtYMTJHhp6bE+zpOD8f5lcm6n14UM74JEVVwCDeogIBQSmNGzX4jkeCuK4VqC2rTZFlSB3DEY55XZfQ2vvcrjG1hfyls5tgUPhq5oI3XYehWhuoOaHYushaRmDLnCG5buNsJGHQFu7/XkHrEGTL5FMBFTKaaYKlxGJy7CVaIJrX794j2ispN9r9XaMgoNxXFHPIM6yGd6UFFwvMJ4YOXHWD3sdAWT+HLWmUxoyP+p29f7uchlma1y+1nGOO5pleCpk2at07Pp+E0pJifUgNg2Khhqa3SJABz";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final double pozitie_cuva_normal = 0.1;

    private void initRobot(){
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");

        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.AUTONOMOUS);
        robot.setMotorPower(0.5);

        motorExtindere = hardwareMap.dcMotor.get("motor_extindere");
        motorExtindere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtindere.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorArticulatie = hardwareMap.dcMotor.get("motor_articulatie");

        motorTijaFiletata = hardwareMap.dcMotor.get("motor_tija_filetata");
        motorTijaFiletata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTijaFiletata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servo_adunare_stanga = hardwareMap.crservo.get("servo_adunare_stanga");
        servo_adunare_dreapta = hardwareMap.crservo.get("servo_adunare_dreapta");
        servo_cuva = hardwareMap.servo.get("servo_cuva");

        motorLift = hardwareMap.dcMotor.get("motor_lift");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    private void coborareRobot(){
       /* mot.setPower(0.66);
        sleep(1700);
        lift.setPower(0);
        sleep(500);

        robot.moveToDirectionCentimeters(10, 0);
        sleep(1000);

        lift.setPower(-0.66);
        sleep(1700);
        lift.setPower(0);
        servo_cuva.setPosition(pozitie_cuva_normal);
        sleep(500);*/
    }

    private void notCoborareRobot(){
        robot.moveToDirectionCentimeters(-10, 0);
        sleep(1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initVuforia();
        initTfod();
        if(tfod != null){
            tfod.activate();
            sleep(500);
        }
        
        waitForStart();
        

        int gold_position = detectieGold();
        telemetry.addData("Gold position", gold_position);
        telemetry.update();
        
        //coborareRobot();
        notCoborareRobot();

        switch (gold_position){
            case 0:
                iaCubMijloc();
                break;
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

    private void iaCubDreapta(){
        robot.rotateToAngle(-60);
        sleep(1250);

        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);

        servo_adunare_dreapta.setPower(1);
        servo_adunare_stanga.setPower(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(40, -90);
        sleep(2500);

        servo_adunare_dreapta.setPower(0);
        servo_adunare_stanga.setPower(0);
        motorArticulatie.setPower(1);
        sleep(600);
        motorArticulatie.setPower(0.11);

        robot.moveToDirectionCentimeters(10, 90);
        sleep(750);

        robot.rotateToAngle(-30);
        sleep(750);

        robot.moveToDirectionCentimeters(100, 180);
        sleep(2000);

        robot.rotateToAngle(40);
        sleep(1250);

        robot.moveToDirectionCentimeters(30,180);
        sleep(1000);

        robot.moveToDirectionCentimeters(155, 90); //distanta 140
        sleep(3000);

        //servo_team_mark.setPosition(-1);
        sleep(2000);


        robot.rotateToAngle(-10);
        sleep(500);
        robot.moveToDirectionCentimeters(180,-90);//165
        sleep(3000);


        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);
    }

    private void iaCubMijloc(){
        robot.rotateToAngle(-95);
        sleep(1000);

        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);

        servo_adunare_dreapta.setPower(1);
        servo_adunare_stanga.setPower(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(35, -90);
        sleep(2500);

        servo_adunare_dreapta.setPower(0);
        servo_adunare_stanga.setPower(0);
        motorArticulatie.setPower(1);
        sleep(600);
        motorArticulatie.setPower(0.11);

        robot.moveToDirectionCentimeters(75, 180);
        sleep(1500);

        robot.rotateToAngle(52.5);
        sleep(1250);

        robot.moveToDirectionCentimeters(43,180);
        //cu 40 trece la limita pe langa mineralul aliat
        sleep(1500);

        robot.moveToDirectionCentimeters(150, 90); //135
        sleep(3000);

        //servo_team_mark.setPosition(-1);
        sleep(2000);


        robot.rotateToAngle(-7);
        sleep(500);

        robot.moveToDirectionCentimeters(170,-90);//165
        sleep(3000);


        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);
    }

    private void iaCubStanga() {
        robot.rotateToAngle(-130);
        sleep(1000);

        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);

        servo_adunare_dreapta.setPower(1);
        servo_adunare_stanga.setPower(-1);
        sleep(500);

        robot.moveToDirectionCentimeters(50, -90);
        sleep(2500);

        servo_adunare_dreapta.setPower(0);
        servo_adunare_stanga.setPower(0);
        motorArticulatie.setPower(1);
        sleep(600);
        motorArticulatie.setPower(0.11);

        robot.moveToDirectionCentimeters(10, 90);
        sleep(750);

        robot.rotateToAngle(85);
        sleep(2000);

        robot.moveToDirectionCentimeters(120, 135);
        sleep(3000);

        robot.moveToDirectionCentimeters(95, 90); //80
        sleep(2000);

        //servo_team_mark.setPosition(-1);
        sleep(2000);


        robot.moveToDirectionCentimeters(170,-90);//155
        sleep(3000);

        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);
    }
}
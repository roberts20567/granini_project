package org.firstinspires.ftc.teamcode.Autonomous;

import android.media.MediaPlayer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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
    private DcMotor motorFrontRight;

    ModernRoboticsI2cRangeSensor rangeSensor;

    private DcMotor motorArticulatie;
    private DcMotor motorLift;
    private DcMotor motorTijaFiletata;
    private DcMotor motorExtindere;

    private CRServo servo_adunare_stanga;
    private CRServo servo_adunare_dreapta;
    private Servo servo_cuva;
    private Servo servo_team_marker;

    private double max_extindere = 2777;
    private double min_extindere = 200;
    private double max_lift = 650;
    private double min_lift = 0;
    private double lift_sigur = 100;

    private static final double cuva_incarcare = 0.70;
    private static final double cuva_descarcare = 0.40;
    private static final double cuva_orizontal = 0.30;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AaWc2RL/////AAABmZlxFvRyrk/YiqLbf3ykKmxLDRhJ5p955zNPuaCd9KvLm88Vfa399ERWzf+8iLlRqzO8q1Rl821vvtYMTJHhp6bE+zpOD8f5lcm6n14UM74JEVVwCDeogIBQSmNGzX4jkeCuK4VqC2rTZFlSB3DEY55XZfQ2vvcrjG1hfyls5tgUPhq5oI3XYehWhuoOaHYushaRmDLnCG5buNsJGHQFu7/XkHrEGTL5FMBFTKaaYKlxGJy7CVaIJrX794j2ispN9r9XaMgoNxXFHPIM6yGd6UFFwvMJ4YOXHWD3sdAWT+HLWmUxoyP+p29f7uchlma1y+1nGOO5pleCpk2at07Pp+E0pJifUgNg2Khhqa3SJABz";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final double pozitie_cuva_normal = 0.1;

    private void initRobot() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");

        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.AUTONOMOUS);
        robot.setMotorPower(1);

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

        servo_team_marker = hardwareMap.servo.get("servo_team_marker");
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
                    } else if (goldMineralX < silverMineral1X) {
                        gold_mineral_position = 2;
                    } else {
                        gold_mineral_position = 3;
                    }
                }
            }
            tfod.shutdown();
        }
        return gold_mineral_position;
    }

    private void mineralScoruit(){
        mineralScoruit(0);
    }

    private void mineralScoruit(int distanta_rotire_bonus){
        mineralScoruit(distanta_rotire_bonus, 0);
    }

    private void mineralScoruit (int distanta_extindere_bonus, int rotire_bouns){

        servo_adunare_dreapta.setPower(1);
        servo_adunare_stanga.setPower(-1);
        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);

        // se fereste cuva de motorArticulatie
        motorLift.setPower(1);
        sleep(75);
        motorLift.setPower(0.2);

        // ia cub in cuva

        motorExtindere.setPower(0.75);
        sleep(1500 + distanta_extindere_bonus);
        motorExtindere.setPower(0);
        sleep(250);
        servo_adunare_dreapta.setPower(0);
        servo_adunare_stanga.setPower(0);

        automatizare_1 = 1;
        motorExtindere.setPower(-1);

        while ((automatizare_1!=0 || automatizare_2!=0) && opModeIsActive()){
            double extindere = -motorExtindere.getCurrentPosition();
            double lift = -motorLift.getCurrentPosition();

            // bratul de extindere se comprima si mineralele sunt basculate in cuva
            if(automatizare_1==1 && extindere<=min_extindere){
                automatizare_1 = 2;
                motorLift.setPower(0);
                servo_cuva.setPosition(cuva_incarcare);
                telemetry.addData("Power lift:", motorLift.getPower());
                telemetry.update();
            }else if(automatizare_1==2 && lift<=min_lift){
                automatizare_1 = 0;
                motorArticulatie.setPower(1);
                sleep(500);
                motorArticulatie.setPower(0);

                motorArticulatie.setPower(-0.5);
                sleep(300);
                motorArticulatie.setPower(0);
                automatizare_2 = 1;
                motorLift.setPower(1);
                telemetry.addLine("Sunt in else..");
                telemetry.update();
            }

            // urca liftul in pozitia de basculare in lander
            if(automatizare_2==1){
                automatizare_2 = 2;
                motorLift.setPower(1);
            }else if(automatizare_2==2 && lift>=max_lift-100){
                automatizare_2 = 3;
                motorLift.setPower(1);
            }else if(automatizare_2==3 && lift>=max_lift){
                automatizare_2 = 0;
                motorLift.setPower(0.2);
            }

            telemetry.addData("Power lift:", motorLift.getPower());
            telemetry.update();
        }

        robot.rotateToAngle(rotire_bouns);
        sleep(50*Math.abs(rotire_bouns));

        robot.moveToDirectionCentimeters(20, 90);
        sleep(500);

        servo_cuva.setPosition(cuva_descarcare);
        sleep(1000);

        robot.moveToDirectionCentimeters(10, -90);
        sleep(750);

        servo_cuva.setPosition(cuva_incarcare);
        motorLift.setPower(0);

        robot.rotateToAngle(-rotire_bouns);
        sleep(50* Math.abs(rotire_bouns));
    }

    private void ridicaFarasSiTine_l(){
        servo_adunare_dreapta.setPower(0);
        servo_adunare_stanga.setPower(0);
        motorArticulatie.setPower(1);
        sleep(600);
        motorArticulatie.setPower(0.11);
    }

    private void aruncaTeamMarker(){
        servo_team_marker.setPosition(-1);
    }

    private int automatizare_1 = 0;
    private int automatizare_2 = 0;

    private void iaCubStanga() {
        robot.moveToDirectionCentimeters(15, 90);
        sleep(1000);

        robot.rotateToAngle(-125);
        sleep(1000);

        mineralScoruit(1000);

        robot.moveToDirectionCentimeters(70, -90);
        sleep(500);

        ridicaFarasSiTine_l();

        robot.moveToDirectionCentimeters(60, -90);
        sleep(1500);

        robot.rotateToAngle(-120);
        sleep(1500);

        robot.moveToDirectionCentimeters(60, 90);
        sleep(1500);

        aruncaTeamMarker();
        sleep(1000);

        robot.moveToDirectionCentimeters(160, -90);
        sleep(2000);

        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);

    }

    private void iaCubMijloc() {
        robot.rotateToAngle(-98);
        sleep(1000);

        mineralScoruit();

        robot.moveToDirectionCentimeters(70, -90);
        sleep(1800);

        ridicaFarasSiTine_l();

        robot.moveToDirectionCentimeters(40, -90);
        sleep(800);

        robot.rotateToAngle(-140);
        sleep(1400);

        robot.moveToDirectionCentimeters(50, 0);
        sleep(800);

        aruncaTeamMarker();
        sleep(2000);


        robot.moveToDirectionCentimeters(167.5, -90);
        sleep(3000);

        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);
    }

    private void iaCubDreapta() {
        robot.rotateToAngle(-68);
        sleep(1000);

        mineralScoruit(1000, -20);

        robot.moveToDirectionCentimeters(70, -90);
        sleep(500);

        ridicaFarasSiTine_l();

        robot.moveToDirectionCentimeters(50, -90);
        sleep(2000);

        robot.rotateToAngle(-180);
        sleep(2000);

        robot.moveToDirectionCentimeters(90, 0);
        sleep(2000);

        aruncaTeamMarker();
        sleep(1000);

        robot.moveToDirectionCentimeters(180, -90);
        sleep(2000);

        motorArticulatie.setPower(-0.5);
        sleep(500);
        motorArticulatie.setPower(0);
    }

    private void coborareRobot(){
        motorTijaFiletata.setTargetPosition(-3940);
        motorTijaFiletata.setPower(1);
        sleep(1700);

        robot.moveToDirectionCentimeters(10, 180);
        sleep(1000);
    }

    private void notCoborareRobot() {
        robot.moveToDirectionCentimeters(-10, 0);
        sleep(1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            sleep(500);
        }

        waitForStart();

        int gold_position = detectieGold();
        telemetry.addData("Gold position", gold_position);
        telemetry.update();

        coborareRobot();
        //notCoborareRobot();

        switch (gold_position) {
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
}
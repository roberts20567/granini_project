package org.firstinspires.ftc.teamcode.Tele_OP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovement;

///cuva 512
/// ridicare pt extindere : 64
///extindere 2777
/////

@TeleOp(name = "USA Teleop")
public class USA_teleop extends OpMode {
    private NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();
    private double viteza_motoare = 0.66;

    private DcMotor motorArticulatie;
    private DcMotor motorLift;
    private DcMotor motorTijaFiletata;
    private DcMotor motorExtindere;

    private CRServo servo_adunare_stanga;
    private CRServo servo_adunare_dreapta;
    private Servo servo_cuva;

    private double max_extindere = 2777;
    private double min_extindere = 200;
    private double max_lift = 700;
    private double min_lift = 0;
    private double lift_sigur = 100;

    private static final double cuva_incarcare = 0.70;
    private static final double cuva_descarcare = 0.35;
    private static final double cuva_orizontal = 0.30;

    @Override
    public void init() {
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setGamepad(gamepad1);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.TELEOP);
        robot.setMotorPower(viteza_motoare);

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
        //motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        robot.opModeLoop();
        controlVitezaMiscare();
        controlArticulatie();
        controlPeri();
        showTelemetry();
        controlLift();
        controlExtindere();
        controlCuva();
        automatizari();
    }

    @Override
    public void stop(){
        robot.stop();
    }

    private void showTelemetry(){
        telemetry.addData("Lift:", motorLift.getCurrentPosition());
        telemetry.addData("Tija Filetata:", motorTijaFiletata.getCurrentPosition());
        telemetry.addData("Extindere:", motorExtindere.getCurrentPosition());
        telemetry.update();
    }

    private void controlVitezaMiscare() {
        robot.setMotorPower(viteza_motoare * (gamepad1.right_trigger + 1) / (gamepad1.left_trigger + 1));
    }

    private int automatizare_1 = 0;
    private int automatizare_2 = 0;
    private int automatizare_3 = 0;

    private int poz_agatare = 0;
    private int poz_temporar = 0;
    private int poz_agatat = 0;

    private void automatizari(){
        double extindere = -motorExtindere.getCurrentPosition();
        double lift = -motorLift.getCurrentPosition();

        // bratul de extindere se comprima si mineralele sunt basculate in cuva
        if(gamepad2.x && automatizare_1==0){
            automatizare_1 = 1;
            motorExtindere.setPower(-1);
        }else if(automatizare_1==1 && extindere<=min_extindere){
            automatizare_1 = 2;
            motorLift.setPower(0);
            servo_cuva.setPosition(cuva_incarcare);
        }else if(automatizare_1==2 && lift<=min_lift){
            automatizare_1 = 0;
            motorArticulatie.setPower(1);
            sleep(500);
            motorArticulatie.setPower(0);

            robot.opModeLoop();
            motorArticulatie.setPower(-0.5);
            sleep(200);
            motorArticulatie.setPower(0);
            automatizare_2 = 1;
            motorLift.setPower(1);
        }

        if(gamepad2.right_stick_y!=0){
            automatizare_1 = 0;
        }

        // se fereste cuva de motorArticulatie
        if(automatizare_1==0 && automatizare_2==0 && extindere>150 && lift<lift_sigur){
            motorLift.setPower(1);
            sleep(75);
            motorLift.setPower(0.2);
        }

        // urca liftul in pozitia de basculare in lander
        if(gamepad2.y && automatizare_2==0){
            automatizare_2 = 1;
            motorLift.setPower(1);
        }else if(automatizare_2==1 && lift>=max_lift-100){
            automatizare_2 = 2;
            motorLift.setPower(0.75);
        }else if(automatizare_2==2 && lift>=max_lift){
            automatizare_2 = 0;
            motorLift.setPower(0.2);
        }

        // coboara liftul
        if(gamepad2.a && automatizare_3==0) {
            automatizare_3 = 1;
            motorLift.setPower(-0.1);
        }else if (automatizare_3==1 && lift<lift_sigur+100){
            automatizare_3 = 0;
            motorLift.setPower(0.5);
            sleep(100);
            motorLift.setPower(0.2);
        }

    }

    private void controlLift(){
        double lift_position = -motorLift.getCurrentPosition();
        double right_stick_y = -gamepad2.right_stick_y;
        if(automatizare_1==0 && automatizare_2==0) {
            if (right_stick_y > 0.2) {//urcare
                if (lift_position < max_lift)
                    motorLift.setPower(right_stick_y);
                else
                    motorLift.setPower(0.2);
            }
            if (right_stick_y < -0.1) {//coborare
                motorLift.setPower(0.02);
            }
        }
    }

    private void controlTijaFiletanta(){
        if (gamepad2.dpad_left){
            motorTijaFiletata.setTargetPosition(poz_agatare);
            motorTijaFiletata.setPower(1); // tija in pozitia de pozitionare
        }
        if (gamepad2.dpad_right){
            motorTijaFiletata.setTargetPosition(poz_temporar);
            motorTijaFiletata.setPower(1); // tija in pozitie de cremaliera intrata

            motorTijaFiletata.setTargetPosition(poz_agatat);
            motorTijaFiletata.setPower(1); // tija in pozitie de agatat
        }


    }


    private void controlArticulatie() {
        if(gamepad2.dpad_up){
            motorArticulatie.setPower(1);
            sleep(500);
            motorArticulatie.setPower(0);
        }
        if(gamepad2.dpad_down){
            motorArticulatie.setPower(-0.5);
            sleep(200);
            motorArticulatie.setPower(0);
        }
        if (gamepad1.right_bumper){
            motorArticulatie.setPower(1);
            sleep(100);
            motorArticulatie.setPower(0);
        }
        if (gamepad1.left_bumper){
            motorArticulatie.setPower(-1);
            sleep(50);
            motorArticulatie.setPower(0);
        }
    }

    private void controlExtindere(){
        if(automatizare_1==0) {
            double extindere = -motorExtindere.getCurrentPosition();
            double left_stick_y = -gamepad2.left_stick_y;
            if (left_stick_y > 0)
                if (extindere < max_extindere)
                    motorExtindere.setPower(left_stick_y);
                else
                    motorExtindere.setPower(0);
            if (left_stick_y <= 0)
                if (extindere > min_extindere)
                    motorExtindere.setPower(left_stick_y);
                else
                    motorExtindere.setPower(0);
        }
    }

    private void controlPeri(){
        double peri_power = gamepad2.right_trigger - gamepad2.left_trigger;
        servo_adunare_dreapta.setPower(peri_power);
        servo_adunare_stanga.setPower(-peri_power);
    }

    private void controlCuva(){
        if(gamepad2.right_bumper){
            servo_cuva.setPosition(cuva_descarcare);
        }
        if(gamepad2.left_bumper){
            servo_cuva.setPosition(cuva_incarcare);
        }
    }

    private void sleep(int x){
        long start_time = System.currentTimeMillis();
        while (System.currentTimeMillis() - start_time < x){
            int a;
        }
    }
}

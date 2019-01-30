package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovement;

@Autonomous(name="Autonomie Timisoara Crater")
public class Autonomie_Timisoara_Crater extends LinearOpMode {

    private NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();
    private DcMotor motorLift;
    private Servo servo_team_mark;
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorRidicare;

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

        motorLift.setPower(0.11);

        waitForStart();
        telemetry.addData("Position BL:" ,motorBackLeft.getCurrentPosition());
        telemetry.addData("Position BR:" ,motorBackRight.getCurrentPosition());
        telemetry.addData("Position FL:" ,motorFrontLeft.getCurrentPosition());
        telemetry.addData("Position FR:" ,motorFrontRight.getCurrentPosition());
        telemetry.update();
        motorLift.setPower(0);
        coborareRobot();
        sleep(2000);
        robot.moveToDirection(400, 90);
        sleep(2000);
        //resetEncoders();
        robot.moveToDirection(800, 0);
        sleep(2000);
        //resetEncoders();
        robot.rotateToAngle(90);
        sleep(4000);

        robot.moveToDirection(2000, -90);
        sleep(3000);

        motorRidicare.setPower(0.5);
        sleep(2000);
        motorRidicare.setPower(0);
        sleep(11);

        /*
        motorFrontRight.setTargetPosition(target);
        motorBackLeft.setTargetPosition(target);
        motorFrontLeft.setTargetPosition(target);
        motorBackRight.setTargetPosition(target);
        motorBackLeft.setPower(0.4);
        motorFrontRight.setPower(0.4);
        motorBackRight.setPower(0.4);
        motorFrontLeft.setPower(0.4);*/


    }

    private void coborareRobot() {

        motorLift.setPower(-0.5);
        sleep(2000);
        motorLift.setPower(-0.11);
        motorRidicare.setPower(-0.75);
        sleep(1000);

    }

    private void resetEncoder(DcMotor motor) {

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private void resetEncoders() {

        resetEncoder(motorBackLeft);
        resetEncoder(motorBackRight);
        resetEncoder(motorFrontLeft);
        resetEncoder(motorFrontRight);

    }

}

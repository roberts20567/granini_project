package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovement;

@Autonomous(name = "default")
public class AutonomieTest extends LinearOpMode {

    private NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        motorBackRight = hardwareMap.dcMotor.get("motor_test_4");

        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.AUTONOMOUS);
        robot.setMotorPower(0.5);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("1", motorFrontRight.getCurrentPosition());
            telemetry.addData("1", motorFrontLeft.getCurrentPosition());
            telemetry.addData("1", motorBackRight.getCurrentPosition());
            telemetry.addData("1", motorBackLeft.getCurrentPosition());
            telemetry.update();
        }


        robot.moveToDirectionCentimeters(50, 0);
        sleep(3000);
    }
}

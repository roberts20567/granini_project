package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiu;
import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovement;

@Autonomous(name="Autonomie Timisoara")
public class Autonomie_Timisoara extends LinearOpMode {

    private NClaudiuOmniDirectionalMovement robot = new NClaudiuOmniDirectionalMovement();

    @Override
    public void runOpMode() throws InterruptedException {
        // <movement>
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor_test_1");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor_test_2");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor_test_3");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor_test_4");
        robot.attachMotors(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovement.DrivingMode.AUTONOMOUS);
        robot.setMotorPower(0.66);
        // </movement>

        waitForStart();
        telemetry.addData("Position BL:" ,motorBackLeft.getCurrentPosition());
        telemetry.addData("Position BR:" ,motorBackRight.getCurrentPosition());
        telemetry.addData("Position FL:" ,motorFrontLeft.getCurrentPosition());
        telemetry.addData("Position FR:" ,motorFrontRight.getCurrentPosition());
        telemetry.update();
        robot.rotateToAngleUsingEncoders(90);
        sleep(1000);
    }
}

package org.firstinspires.ftc.teamcode.Tele_OP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.clasele_lui_claudiu.NClaudiuOmniDirectionalMovementPro;

@TeleOp(name = "Gratiela")
public class Gratiela extends OpMode {
    private NClaudiuOmniDirectionalMovementPro robot = new NClaudiuOmniDirectionalMovementPro();

    @Override
    public void init() {
        DcMotor motor_1 = hardwareMap.dcMotor.get("motor1");
        DcMotor motor_2 = hardwareMap.dcMotor.get("motor2");
        DcMotor motor_3 = hardwareMap.dcMotor.get("motor3");
        DcMotor motor_4 = hardwareMap.dcMotor.get("motor4");
        robot.attachMotor(motor_1);
        robot.attachMotor(motor_2);
        robot.attachMotor(motor_3);
        robot.attachMotor(motor_4);

        robot.setGamepad(gamepad1);
        robot.setMotorsPower(0.5);
        robot.setRobotFrontSideAngle(-45);
        robot.setDrivingMode(NClaudiuOmniDirectionalMovementPro.DrivingMode.TELEOP);
        robot.init();
    }

    @Override
    public void loop() {
        robot.opModeLoop();
    }
}

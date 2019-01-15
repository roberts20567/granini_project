package org.firstinspires.ftc.teamcode.clasele_lui_claudiu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Xeo18-19: Claudiu test", group = "Xeo18-19")
public class test_claudiu extends OpMode {
    OmniDirectionalMovement omni;

    @Override
    public void init(){
        omni = new OmniDirectionalMovement(hardwareMap, "motor_test_1", "motor_test_2", "motor_test_3", "motor_test_4");
        omni.setMotorPower(1);
    }

    @Override
    public void loop(){
        if(gamepad1.y){
            omni.calibratePow(123);
        }
        omni.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

}

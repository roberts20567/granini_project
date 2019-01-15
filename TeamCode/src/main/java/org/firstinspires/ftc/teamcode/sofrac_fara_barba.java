package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Xeo18-19: S", group = "Xeo18-19")
public class sofrac_fara_barba extends OpMode {
    private DcMotor motor_intindere;
    private double viteza;
    private double viteza_1 = 1;
    private double viteza_2 = 0.5;
    final int a = 9;
    @Override
    public void init() {
    motor_intindere = hardwareMap.dcMotor.get("motor_intindere");
    viteza = viteza_1;
    }

    @Override
    public void loop(){
        double power = gamepad2.right_stick_y;
        motor_intindere.setPower(power * viteza);
    if(gamepad2.a){
        viteza = viteza_1;
    }
    else if (gamepad2.b){
        viteza = viteza_2;
    }

    }

    @Override
    public void stop(){
        motor_intindere.setPower(0);
    }
}

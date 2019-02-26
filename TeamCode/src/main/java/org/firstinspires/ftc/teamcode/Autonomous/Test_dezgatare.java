package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Test dezgatare")
public class Test_dezgatare extends LinearOpMode {

    private DcMotor motorLift;
    private DcMotor frateMotorLift;
    private Lift lift;

    private void initRobot(){
        motorLift = hardwareMap.dcMotor.get("motor_lift");
        frateMotorLift = hardwareMap.dcMotor.get("frate_motor_lift");
        lift = new Lift(motorLift, frateMotorLift);
        lift.setPower(-0.11);
    }



    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        lift.setPower(0.66);
        sleep(1700);

    }
}

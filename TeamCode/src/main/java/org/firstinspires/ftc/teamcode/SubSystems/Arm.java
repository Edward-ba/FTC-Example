package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm implements SubSystem {
double x;
    private final Config config;
    private DcMotor armMotor;

    public Arm(Config cfg){
        this.config = cfg;
    }

    @Override
    public void init() {
        armMotor = config.hardwareMap.get(DcMotor.class, Config.ARM_MOTOR);
        // Reset the encoder and set it to be in RUN_TO_POSITION
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        double armpower = config.gamePad2.left_stick_y;  // Note: pushing stick forward gives negative value
        armMotor.setPower(armpower*3/4);
    }
}
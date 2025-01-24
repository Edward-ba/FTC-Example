package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake implements SubSystem {

    private final Config config;
    private CRServo intake;

    public Intake(Config cfg){
        this.config = cfg;
    }

    @Override
    public void init() {
        intake = config.hardwareMap.get(CRServo.class, Config.INTAKE);
        // Reset the encoder and set it to be in RUN_TO_POSITION
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        if (config.gamePad2.right_bumper) {
            intake.setPower(3.0);
        }
        else if (config.gamePad2.left_bumper) {
            intake.setPower(-3.0);
        }
        else {
            intake.setPower(0.0);
        }
    }
}
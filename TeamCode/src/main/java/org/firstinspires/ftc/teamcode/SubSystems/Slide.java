package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Slide implements SubSystem {

    private final Config config;
    private DcMotor slideMotor;

    public Slide(Config cfg){
        this.config = cfg;
    }

    @Override
    public void init() {
        slideMotor = config.hardwareMap.get(DcMotor.class, Config.SLIDE_MOTOR);
        // Reset the encoder and set it to be in RUN_TO_POSITION
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update() {
        double slidepower = config.gamePad2.right_stick_y;  // Note: pushing stick forward gives negative value
        slideMotor.setPower(slidepower/2);
    }
}
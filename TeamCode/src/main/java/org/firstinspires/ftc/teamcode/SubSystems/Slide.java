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
        if (config.gamePad2.x) {
            slideMotor.setPower(1);
        }
        else if (config.gamePad2.y) {
            slideMotor.setPower(-1);
        }
        else {
            slideMotor.setPower(0);
        }
    }
}
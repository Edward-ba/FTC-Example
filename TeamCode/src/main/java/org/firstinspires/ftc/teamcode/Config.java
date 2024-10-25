package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.zip.ZipEntry;

public class Config {
    public Telemetry telemetry = null;
    public HardwareMap hardwareMap = null;
    public Gamepad gamePad1 = null;
    public Gamepad gamePad2 = null;

    // Drive system
    public static final String RIGHT_FRONT_DRIVE = "frontright";
    public static final String RIGHT_BACK_DRIVE = "backright";
    public static final String LEFT_FRONT_DRIVE = "frontleft";
    public static final String LEFT_BACK_DRIVE = "backleft";

    public static final String SLIDE_MOTOR = "slide";
    public static final String ARM_MOTOR = "motorarm";

    private ElapsedTime runtime = new ElapsedTime();

    // Constructor
    public Config(Telemetry tlm, HardwareMap hwm, Gamepad gmp1, Gamepad gmp2) {
        this.telemetry = tlm;
        this.hardwareMap = hwm;
        this.gamePad1 = gmp1;
        this.gamePad2 = gmp2;
    }

    void updateTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("G1: bumper", "%b %b", gamePad1.left_bumper, gamePad1.right_bumper);
        telemetry.addData("G1: trigger", "%4.2f, %4.2f", gamePad1.left_trigger, gamePad1.right_trigger);
    }
}

// mostly const variables, and some stuff you don't need to worry about, if you need to add more motors/servos, just add them under SLIDE_MOTOR
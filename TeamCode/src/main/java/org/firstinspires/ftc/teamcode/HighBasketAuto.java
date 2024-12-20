package org.firstinspires.ftc.teamcode;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@Autonomous(name = "High Basket Auto Test", group = "Autonomous")
public class HighBasketAuto extends LinearOpMode {

    public class Lift {
        private DcMotorEx lift;
        private DcMotorEx slide;
        private CRServo cervo;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, org.firstinspires.ftc.teamcode.Config.ARM_MOTOR);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            slide = hardwareMap.get(DcMotorEx.class, org.firstinspires.ftc.teamcode.Config.SLIDE_MOTOR);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setDirection(DcMotorSimple.Direction.REVERSE);
            cervo = hardwareMap.get(CRServo.class, org.firstinspires.ftc.teamcode.Config.INTAKE);
            cervo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                System.out.println(pos);
                if (pos < 1850) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                System.out.println(pos);
                if (pos > 5500) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
        public class slideOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(0.8);
                    initialized = true;
                }

                double pos = slide.getCurrentPosition();
                packet.put("slidePos", pos);
                System.out.println(pos);
                if (pos < 800) {
                    return true;
                } else {
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideOut(){
            return new slideOut();
        }

        public class slideIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(-0.8);
                    initialized = true;
                }

                double pos = slide.getCurrentPosition();
                packet.put("slidePos", pos);
                System.out.println(pos);
                if (pos > -500) {
                    return true;
                } else {
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideIn(){
            return new slideIn();
        }
        public class spitOut implements Action {
            private boolean initialized = false;
            double pos1 = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    cervo.setPower(-0.8);
                    initialized = true;
                }
                pos1 = pos1 + 1;
                packet.put("slidePos", pos1);
                System.out.println(pos1);
                if (pos1 < 7000) {
                    return true;
                } else {
                    cervo.setPower(0);
                    return false;
                }
            }
        }
        public Action spitOut(){
            return new spitOut();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-28, 14))
                .turn(Math.PI/4+Math.PI/9);
                //.strafeTo(new Vector2d(-18, 0))
                ;

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        lift.slideIn(),
                        tab1.build(),
                        lift.liftUp(),
                        lift.slideOut(),
                        lift.spitOut()
                )
        );

    }
}





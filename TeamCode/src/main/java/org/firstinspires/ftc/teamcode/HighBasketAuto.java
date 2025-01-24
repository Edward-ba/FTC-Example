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
        private CRServo servo;
        private double startposl;
        private double startposs;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, org.firstinspires.ftc.teamcode.Config.ARM_MOTOR);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            System.out.println("initial lift position" + lift.getCurrentPosition());
            startposl = lift.getCurrentPosition();
            slide = hardwareMap.get(DcMotorEx.class, org.firstinspires.ftc.teamcode.Config.SLIDE_MOTOR);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setDirection(DcMotorSimple.Direction.REVERSE);
            System.out.println("initial slide position" + slide.getCurrentPosition());
            startposs = slide.getCurrentPosition();
            servo = hardwareMap.get(CRServo.class, org.firstinspires.ftc.teamcode.Config.INTAKE);
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }
                /*double limit1l;
                if (startposl > 0) {
                    limit1l = startposl;
                }else{
                    limit1l = -startposl;
                }*/
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                //intiial is -25, limit of 1850 only lifts 1/3, limit of 5000 was really low
                //System.out.println("pos-startposl = ");
                //System.out.println(pos-startposl);
                if (pos - startposl< 5200) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {return new LiftUp();}

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                System.out.println(pos-startposl);
                packet.put("liftPos", pos);
                //System.out.println("pos-startposl2 = ");
                //System.out.println(pos-startposl);
                if (pos - startposl> 3700) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action LiftDown() {return new LiftDown();}

        public class slideOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(0.8);
                    initialized = true;
                }
                double pos_s = slide.getCurrentPosition();
                packet.put("slidePos", pos_s);
                //System.out.println("pos_s - startposs = ");
                //System.out.println(pos_s - startposs);
                //initial postion -19, 400 is d, 1500 was too short
                if (pos_s - startposs < 1799) {
                    return true;
                } else {
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideOut() {return new slideOut();}

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
                //System.out.println("pos_s - startposs2 = ");
                //System.out.println(pos-startposs);
                if (pos-startposs > -150) {
                    return true;
                } else {
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideIn() {return new slideIn();}


        public class Intaker implements Action {
            long dt;
            double power;

            public Intaker(long dt, double power) {
                this.dt = dt;
                this.power = power;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPower(power);
                sleep(dt);
                servo.setPower(0);
                return false;
            }
        }
        public Action intake(long dt, double power) {return new Intaker(dt, power);}
        public class LiftDown2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                System.out.println(pos-startposl);
                packet.put("liftPos", pos);
                System.out.println("pos-startposl2 = ");
                System.out.println(pos-startposl);
                if (pos - startposl> 1500) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action LiftDown2() {return new LiftDown2();}
        public class slideOut2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(0.8);
                    initialized = true;
                }
                double pos_s = slide.getCurrentPosition();
                packet.put("slidePos", pos_s);
                //System.out.println("pos_s - startposs = ");
                //System.out.println(pos_s - startposs);
                //initial postion -19, 400 is d, 1500 was too short
                if (pos_s - startposs < 1200) {
                    return true;
                } else {
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideOut2() {return new slideOut2();}
    }

    public class movebar implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
            TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(-1, -1))
                    .turn(Math.PI/4-Math.PI/9-Math.PI/2);
            return false;
        }
    }

    public Action movebar() {return new movebar();}

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        TrajectoryActionBuilder drivetobasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-30, 16))
                .turn(Math.PI/4+Math.PI/9);
        TrajectoryActionBuilder turntofaceforward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-1, -1))
                .turn(-Math.PI/4-Math.PI/9-Math.PI/3.3);
        TrajectoryActionBuilder drivetobar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 10))
                .strafeTo(new Vector2d(-60, 10))
                .strafeTo(new Vector2d(-60, 27))
                //.turn(-Math.PI/4-Math.PI/9-Math.PI/5);
                //.strafeTo(new Vector2d(-18, 0))
                ;
        TrajectoryActionBuilder turntofacebackward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-1, -1))
                .turn(-Math.PI/10);
        waitForStart();
        TrajectoryActionBuilder drivetosample = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, -13))
                .strafeTo(new Vector2d(31.5, -13))
                .strafeTo(new Vector2d(31.5, -7))
                //.turn(-Math.PI/4-Math.PI/9-Math.PI/5);
                //.strafeTo(new Vector2d(-18, 0))
                ;
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 0))
                .turn((Math.PI/4+Math.PI/9)*2/3)
                ;
        TrajectoryActionBuilder tab7 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(20, 30))
                .strafeTo(new Vector2d(20, 20))
                .strafeTo(new Vector2d(2, -20))
                .turn((Math.PI/4+Math.PI/9)*2/3)
                ;
        TrajectoryActionBuilder tab8 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 19))
                ;
        TrajectoryActionBuilder drivetobasket2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, -6))
                .strafeTo(new Vector2d(-31.5, -6))
                .strafeTo(new Vector2d(-31.5, 13))
                //.turn(-Math.PI/4-Math.PI/9-Math.PI/5);
                //.strafeTo(new Vector2d(-18, 0))
                ;
        TrajectoryActionBuilder turntofacebackward2 = drive.actionBuilder(initialPose)
                //.strafeTo(new Vector2d(-1, -1))
                .turn(Math.PI/10);
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        lift.slideIn(),
                        drivetobasket.build(),
                        lift.liftUp(),
                        lift.slideOut(),
                        lift.intake(500, -0.8),
                        lift.slideIn(),
                        //turntofaceforward.build(),
                        //drivetobar.build(),
                        //lift.LiftDown()
                        turntofacebackward.build(),
                        drivetosample.build(),
                        tab6.build(),
                        tab7.build(),
                        tab8.build(),
                        lift.LiftDown()
                        //lift.intake(800, 1),
                        //lift.slideIn(),
                        //lift.liftUp(),
                        //drivetobasket2.build(),
                        //turntofacebackward2.build()
                )
        );
    }
}





package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

@Autonomous(name = "RedLeftAuto", group = "Autonomous")
public class RedLeftAuto extends LinearOpMode {

    public class Arm {
        private DcMotorEx armMotor;

        public Arm() {
            armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class MoveArm implements Action {
            private int targetPosition;
            private boolean initialized = false;

            public MoveArm(int targetPosition) {
                this.targetPosition = targetPosition;
                new SleepAction(1);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setTargetPosition(targetPosition);
                    armMotor.setPower(1.0);
                    initialized = true;
                }
                packet.put("Arm Position", armMotor.getCurrentPosition());
                return armMotor.isBusy();
            }

        }

        public Action moveToPosition(int position) {
            return new MoveArm(position);
        }
    }

    public class Claw {
        private Servo claw1, claw2;

        public Claw() {
            claw1 = hardwareMap.get(Servo.class, "claw1");
            claw2 = hardwareMap.get(Servo.class, "claw2");
        }

        public class SetClaw implements Action {
            private double position1, position2;
            private boolean initialized = false;

            public SetClaw(double position1, double position2) {
                this.position1 = position1;
                this.position2 = position2;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    claw1.setPosition(position1);
                    claw2.setPosition(position2);
                    initialized = true;
                }
                packet.put("Claw1 Position", claw1.getPosition());
                packet.put("Claw2 Position", claw2.getPosition());
                return false; // Action completes immediately
            }
        }

        public Action open() {
            return new SequentialAction(
                    new SetClaw(0.0, 0.8), // Open claw to predefined positions
                    new SleepAction(1)     // Wait for 1 second
            );
        }

        public Action close() {
            return new SequentialAction(
                    new SetClaw(0.4, 0.2), // Close claw to predefined positions
                    new SleepAction(1)     // Wait for 1 second
            );
        }
    }


    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-23.9, -71.39, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Arm arm = new Arm();
        Claw claw = new Claw();

        // Goes to first sample
        Action trajectory0 = drive.actionBuilder(new Pose2d(-23.9, -71.39, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-42,-46))
                .waitSeconds(1)
                .turn(Math.toRadians(-180))
                .waitSeconds(1)
                .lineToY(-46)
                .build();

        // Goes to basket to score first sample
        Action trajectory1 = drive.actionBuilder(new Pose2d(-42, -47, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-50, -50))
                .waitSeconds(1)
                .turn(Math.toRadians(-220))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-57, -64))
                .build();

        // Goes to second sample
        Action trajectory2 = drive.actionBuilder(new Pose2d(-55, -62, Math.toRadians(-310)))
                .strafeTo(new Vector2d(-56,-46))
                .waitSeconds(1)
                .turn(Math.toRadians(-130))
                .waitSeconds(1)
                .lineToY(-44.5)
                .waitSeconds(1)
                .build();

        // Goes to basket and scores second sample
        Action trajectory3 = drive.actionBuilder(new Pose2d(-56, -44.5, Math.toRadians(-710)))
                .strafeTo(new Vector2d(-50, -50))
                .waitSeconds(1)
                .turn(Math.toRadians(-160))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-58, -62))
                .build();

        // Parks
        Action trajectory4 = drive.actionBuilder(new Pose2d(-58, -62, Math.toRadians(-710-160)))
                .strafeTo(new Vector2d(-50,-50))
                .waitSeconds(1)
                .strafeTo(new Vector2d(70,-70))
                .build();

        // Parks
        Action trajectory5 = drive.actionBuilder(new Pose2d(-57, -64, Math.toRadians(-220)))
                .strafeTo(new Vector2d(-50,-50))
                .waitSeconds(1)
                .strafeTo(new Vector2d(68,-68))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Execute the trajectories and actions
        Actions.runBlocking(
                new SequentialAction(
                        trajectory0,
                        arm.moveToPosition(ArmPresets.ARM_COLLECT_SAMPLE),
                        claw.close(),
                        arm.moveToPosition(ArmPresets.ARM_SCORE_SAMPLE_LOW),
                        trajectory1,
                        claw.open(),
                        //trajectory2,
                        //arm.moveToPosition(ArmPresets.ARM_COLLECT_SAMPLE),
                        //claw.close(),
                        //arm.moveToPosition(ArmPresets.ARM_SCORE_SAMPLE_LOW),
                        //trajectory3,
                        //claw.open(),
                        arm.moveToPosition(ArmPresets.ARM_COLLAPSED),
                        //trajectory4,
                        trajectory5
                )
        );

    }
}
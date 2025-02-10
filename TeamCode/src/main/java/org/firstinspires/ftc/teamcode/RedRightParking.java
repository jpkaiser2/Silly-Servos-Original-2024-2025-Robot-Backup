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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

@Autonomous(name = "RedRightParking", group = "Autonomous")
public class RedRightParking extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(23.90, -71.39, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Arm arm = new Arm();
        Claw claw = new Claw();

        // Comes out
        Action trajectory0 = drive.actionBuilder(new Pose2d(23.90, -71.39, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(23.90,-67))
                .waitSeconds(1)
                .build();

        // Parks
        Action trajectory1 = drive.actionBuilder(new Pose2d(23.90, -67, Math.toRadians(90)))
                .strafeTo(new Vector2d(60, -68))
                .waitSeconds(1)
                .build();



        waitForStart();
        if (isStopRequested()) return;

        // Execute the trajectories and actions
        Actions.runBlocking(
                new SequentialAction(
                        trajectory0,
                        trajectory1
                )
        );

    }
}
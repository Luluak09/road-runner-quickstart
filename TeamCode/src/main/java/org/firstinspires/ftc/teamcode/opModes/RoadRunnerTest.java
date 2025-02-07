package org.firstinspires.ftc.teamcode.opModes;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.*;

import java.lang.Math;

@Autonomous
public class RoadRunnerTest extends LinearOpMode {
    public class CustomAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double gripLiftPower = (targetLift-gripLift.getCurrentPosition())*kp;
            gripLift.setPower(gripLiftPower);

            return true;


        }
    }
    public class TargetAction implements Action {
        public int inputTarget;
        public TargetAction(int target) {
            inputTarget = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetLift = inputTarget;
            return false;


        }
    }

    public class ServoAction implements Action {
        public double inputTarget;
        public Servo inputservo;
        public ServoAction(double target, Servo servo) {
            inputTarget = target;
            inputservo = servo;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            inputservo.setPosition(inputTarget);
            return false;


        }
    }
    DcMotor gripLift;
    Servo claw;
    int targetLift = 0;
    double kp = 0.01;
    @Override
    public void runOpMode() {

        gripLift = hardwareMap.get(DcMotor.class, "gripLift");
        Pose2d startPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,startPose);

        gripLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gripLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        gripLift.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));
claw = hardwareMap.get(Servo.class,"clawServo");


        TrajectoryActionBuilder t1 = drive.actionBuilder(new Pose2d(0,0,0))
                        .setTangent(0)

                                .splineToConstantHeading(new Vector2d(32,4),PI);
        Action forward = t1.build();

        TrajectoryActionBuilder t2 = drive.actionBuilder(new Pose2d(32,4,PI/2))
                .setTangent(PI/2)

                .splineToConstantHeading(new Vector2d(-13,-22),PI/2);
        Action backwards = t2.build();
claw.setPosition(0.2);
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                new CustomAction(),

                        new SequentialAction(
                                new ServoAction(0.2,claw),
                                new TargetAction(2000), forward,
                                new TargetAction(1500),new SleepAction(3),
                                new ServoAction(0.5, claw),backwards
                        )
                )
        );





            }
        }


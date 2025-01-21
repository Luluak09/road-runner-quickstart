package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.*;

import java.lang.Math;


public class RoadRunnerTest extends LinearOpMode {
    @Override 
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));
        
        drive.setPositionEstimate(startPose);


Trajectory traj1 = drive.trajectoryBuilder(startPose)
        .splineTo(new vector2d(40, 40), Math.toRadians(45))
        .build();

        Trajectory traj = drive.trajectoryBuilder(traj1.end())
                .splineTo(new vector(40, 40), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj1);
        drive.Trajectory(traj2);
        
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        
        
            myVector = new Vector2d(10, -5);

    }
}


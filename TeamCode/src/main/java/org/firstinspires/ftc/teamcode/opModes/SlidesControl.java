package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class SlidesControl extends LinearOpMode {
    DcMotor slides;

   public static int target = 0;
   public static double kp = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = hardwareMap.get(DcMotor.class,"gripLift");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        slides.setDirection(DcMotorSimple.Direction.REVERSE);



      waitForStart();
      while(opModeIsActive()){

          double error = target-slides.getCurrentPosition();
          double motorPower = kp * error;
          slides.setPower(motorPower);
            tele.addData("target", target);
            tele.addData("position", slides.getCurrentPosition());
            tele.addData("error", error);
            tele.addData("power", motorPower);
            tele.update();
      }





    }
}

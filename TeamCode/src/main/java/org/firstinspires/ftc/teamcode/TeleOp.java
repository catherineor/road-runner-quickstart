// Marlbots-2020-2021-TeleOp-Code

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="TeleOp", group="Iterative Opmode")

public class TeleOpMeets extends OpMode
{
    //shooter
    private DcMotor shooterWheel;
    private Servo shooterFlicker;
    boolean changeFlicker = true;
    private float flickerPos = 0.5f;

    //indexing
    private DcMotor index;
    private Servo indexRight;
    private Servo indexLeft;

    //intake
    private DcMotor intake;

    //wobble goal
    private Servo wobbleDolly;
    private Servo wobblePivotTop;
    private Servo wobblePivotBottom;
    private Servo wobbleClawTop;
    private Servo wobbleClawBottom;

    //Drivetrain
    private DcMotor driveFrontRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackRight;
    private DcMotor driveBackLeft;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //indexing
        index = hardwareMap.get(DcMotor.class, "index");
        //indexRight = hardwareMap.get(Servo.class, "indexRight");
        //indexLeft = harwareMap.get(Servo.class, "indexLeft");

        //shooter
        shooterWheel = hardwareMap.get(DcMotor.class, "shooterWheel");
        shooterFlicker = hardwareMap.get(Servo.class, "shooterFlicker");
        shooterFlicker.setPosition(flickerPos);

        //intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        //wobble goal
        wobbleDolly = hardwareMap.get(Servo.class, "wobbleDolly");
        wobblePivotTop = hardwareMap.get(Servo.class, "wobblePivotTop");
        wobblePivotBottom = hardwareMap.get(Servo.class, "wobblePivotBottom");
        wobbleClawTop = hardwareMap.get(Servo.class, "wobbleClawTop");
        wobbleClawBottom = hardwareMap.get(Servo.class, "wobbleClawBottom");

        //Drivetrain
        driveFrontRight = hardwareMap.get(DcMotor.class,"driveFrontRight");
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {
        //intake
        if(gamepad2.right_bumper){
            intake.setPower(1);
        }
        else if(gamepad2.left_bumper){
            intake.setPower(-1);
        }
        else{
            intake.setPower(0);
        }

        //indexing
        if(gamepad2.right_trigger>0)
        {
            index.setPower(-1);
        }
        else
        {
            index.setPower(0);
        }
    /*//if(gamepad2.b && indexLeft.getCurrentPosition == 0 && indexRight.getCurrentPosition == 1)
    {
      indexLeft.setPosition(.5); //check negative or positive (one is clockwise, other counter)
      indexRight.setPosition(.5);
    }
    else if (gamepad2.b && indexLeft.setPosition(.5) && indexRight.setPosition(.5))
    {
      indexLeft.setPosition(0);
      indexRight.setPosition(1);
    }*/

        //shooter
        if(gamepad2.left_trigger>0)
        {
            shooterWheel.setPower(-0.67);
        }
        else
        {
            shooterWheel.setPower(0);
        }

        if(gamepad2.a && changeFlicker)
        {
            flickerPos = 1;
            shooterFlicker.setPosition(flickerPos);
            changeFlicker = !changeFlicker;
        }
        else if(gamepad2.a && !changeFlicker)
        {
            flickerPos = .5f;
            shooterFlicker.setPosition(flickerPos);
            changeFlicker = !changeFlicker;
        }

        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        //loops over and over again, put if statements that control motors and servos based on button presses
        //while (opModeIsActive()) {
        //Drivetrain

        // slightly adjusted drivetrain code from 2019-2020
        // drivetrain wheel variables
        float drive = gamepad1.left_stick_x;
        float strafe = gamepad1.left_stick_y;
        float turn = gamepad1.right_stick_x;

        // uses variables to set power
        driveFrontRight.setPower(drive - strafe + turn); //would have to check which is + and -
        driveFrontLeft.setPower(-drive - strafe - turn);
        driveBackRight.setPower(-drive - strafe + turn);
        driveBackLeft.setPower(drive - strafe - turn);
        //}

        //wobble goal - need to finish
        if (gamepad1.left_bumper)
        {
            wobbleClawBottom.setPosition(1);
        }
        else
        {
            wobbleClawBottom.setPosition(0);
        }
        if (gamepad1.right_bumper)
        {
            wobbleClawTop.setPosition(1);
        }
        else
        {
            wobbleClawTop.setPosition(0);
        }



    }
}

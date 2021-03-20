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


@TeleOp(name="TeleOpFlicker", group="Iterative Opmode")

public class TeleOpFlicker extends OpMode
{
    //shooter
    private DcMotor shooterWheel;
    private Servo shooterFlicker;

    //indexing
    private DcMotor index;
    private Servo indexRight;
    private Servo indexLeft;
    boolean doorOpen = true;

    //intake
    private DcMotor intake;

    //wobble goal
    private DcMotor wobbleLead;
    private Servo wobblePivotTop;
    private Servo wobblePivotBottom;
    private Servo wobbleClaw;
    boolean clawOpen = false;
    boolean inBot=true;

    //Drivetrain
    private DcMotor driveFrontRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackRight;
    private DcMotor driveBackLeft;
    boolean halfPower = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //indexing
        index = hardwareMap.get(DcMotor.class, "index");
        indexRight = hardwareMap.get(Servo.class, "indexRight");
        indexLeft = hardwareMap.get(Servo.class, "indexLeft");
        indexLeft.setPosition(0);
        indexRight.setPosition(1);

        //shooter
        shooterWheel = hardwareMap.get(DcMotor.class, "shooterWheel");
        shooterFlicker = hardwareMap.get(Servo.class, "shooterFlicker");
        shooterFlicker.setPosition(0);

        //intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        //wobble goal
        wobbleLead = hardwareMap.get(DcMotor.class, "wobbleLead");
        wobblePivotTop = hardwareMap.get(Servo.class, "wobblePivotTop");
        wobblePivotBottom = hardwareMap.get(Servo.class, "wobblePivotBottom");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        wobblePivotTop.setPosition(0);
        wobblePivotBottom.setPosition(1);
        wobbleClaw.setPosition(1);
        

        //Drivetrain
        driveFrontRight = hardwareMap.get(DcMotor.class,"driveFrontRight");
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {
        //intake and indexing
        if(gamepad2.left_bumper){
            intake.setPower(-1);
            index.setPower(1);
        }
        else if(gamepad2.right_bumper)
        {
            intake.setPower(1);
            index.setPower(-1);
        }
        intake.setPower(0);
        index.setPower(0);

        //shooter
        if(gamepad2.left_trigger>0)
        {
            shooterWheel.setPower(-.48);
        }
        else if(gamepad2.right_trigger>0)
        {
            shooterWheel.setPower(-.25);
        }
        shooterWheel.setPower(0);
        
        while(gamepad2.a)
        {
            shooterFlicker.setPosition(1);
        }
        shooterFlicker.setPosition(0);

        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        shooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Drivetrain
        float drive = gamepad1.left_stick_x;
        float strafe = gamepad1.left_stick_y;
        float turn = gamepad1.right_stick_x;

        if(gamepad1.a)
        {
            halfPower=!halfPower;
        }
        if (!halfPower) {
            driveFrontRight.setPower(drive - strafe + turn);
            driveFrontLeft.setPower(-drive - strafe - turn);
            driveBackRight.setPower(-drive - strafe + turn);
            driveBackLeft.setPower(drive - strafe - turn);
        }
        else if(halfPower)
        {
            driveFrontRight.setPower((drive - strafe + turn)/2);
            driveFrontLeft.setPower((-drive - strafe - turn)/2);
            driveBackRight.setPower((-drive - strafe + turn)/2);
            driveBackLeft.setPower((drive - strafe - turn)/2);
        }

        //wobble goal
        if(gamepad2.left_stick_y>0)
        {
            wobbleLead.setPower(1);
        }
        else if(gamepad2.left_stick_y<0)
        {
            wobbleLead.setPower(-1);
        }
        wobbleLead.setPower(0);
        
        if (gamepad2.x && clawOpen)
        {
            wobbleClaw.setPosition(1);
            clawOpen=!clawOpen;
        }
        if(gamepad2.x && !clawOpen)
        {
            wobbleClaw.setPosition(.7);
            clawOpen=!clawOpen;
        }
        
        if (gamepad2.y && inBot)
        {
            wobblePivotTop.setPosition(1);
            wobblePivotBottom.setPosition(0);
            inBot=!inBot;
        }
        else if (gamepad2.y && !inBot)
        {
            wobblePivotTop.setPosition(0);
            wobblePivotBottom.setPosition(1);
            inBot=!inBot;
        }
        
        if(gamepad2.b && doorOpen)
        {
            indexRight.setPosition(.75);
            indexLeft.setPosition(.34);
            doorOpen=!doorOpen;
        }
        else if(gamepad2.b && !doorOpen)
        {
            indexRight.setPosition(1);
            indexLeft.setPosition(0);
            doorOpen=!doorOpen;
        }
    }
}

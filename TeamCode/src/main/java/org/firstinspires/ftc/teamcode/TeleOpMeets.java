// Marlbots-2020-2021-TeleOp

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name="TeleOpMeets", group="Iterative Opmode")

public class TeleOpMeets extends OpMode
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
    private Servo wobbleThirdPivot;
    private Servo wobbleClaw;
    //combined buttons
    private boolean pressed1 = false;
    private boolean done1 = false;
    private boolean pressed2 = false;
    private boolean done2 = false;
    private boolean pressed3 = false;
    private boolean done3 = false;
    ElapsedTime timer = new ElapsedTime();

    //Drivetrain
    private DcMotor driveFrontRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackRight;
    private DcMotor driveBackLeft;
    boolean halfPower = false;
    
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    ElapsedTime runtime = new ElapsedTime();

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
        shooterFlicker.setPosition(.3);

        //intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        //wobble goal
        wobbleLead = hardwareMap.get(DcMotor.class, "wobbleLead");
        wobblePivotTop = hardwareMap.get(Servo.class, "wobblePivotTop");
        wobblePivotBottom = hardwareMap.get(Servo.class, "wobblePivotBottom");
        wobbleThirdPivot = hardwareMap.get(Servo.class, "wobbleThirdPivot");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        wobblePivotTop.setPosition(0);
        wobblePivotBottom.setPosition(1);
        wobbleThirdPivot.setPosition(0);
        wobbleClaw.setPosition(1);

        //Drivetrain
        driveFrontRight = hardwareMap.get(DcMotor.class,"driveFrontRight");
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");

        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        shooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("claw pos:", wobbleClaw.getPosition());
        telemetry.update();
    }
    @Override
    public void loop() {
        
        //just claw
        // && wobbleClaw.getPosition()>0.5 && wobbleClaw.getPosition()<.8)
        if (gamepad1.b || gamepad2.dpad_left)
        {
            wobbleClaw.setPosition(1);
        }
        // && wobbleClaw.getPosition()>0.8 && wobbleClaw.getPosition()<=1)
        else if(gamepad1.y || gamepad2.dpad_right)
        {
            wobbleClaw.setPosition(.75);
        }
        
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
        else
        {
            intake.setPower(0);
            index.setPower(0);
        }
        //automate power shots
        if(gamepad2.y)
        {
            resetEncoders();
            useEncoders();
            encoderCrab(10,.6);
        }
        /*if (gamepad2.y && pressed3 == false)
        {
            done3 = false;
            if (!pressed3)
            {
                timer.reset();
                pressed3 = true;
            }
        }
        else if (pressed3 == true && done3 == false) {
            //shooterWheel.setPower(-.45);
            if (timer.seconds() > 0 && timer.seconds() < .5)
            {
                flicker();
            }
            if (timer.seconds() > .5 && timer.seconds() < .8)
            {
                //shooterWheel.setPower(-.41);
                turn(.5);
            }
            if (timer.seconds() > .8 && timer.seconds() < 1.3)
            {}
            if (timer.seconds() > 1.3 && timer.seconds() < 1.8) {
                flicker();
            }
            if (timer.seconds() > 1.8 && timer.seconds() < 2.2)
            {
                //shooterWheel.setPower(-.4);
                turn(-.5);
            }
            if (timer.seconds() > 2.3 && timer.seconds() < 2.8)
            {}
            if (timer.seconds() > 2.8 && timer.seconds() < 3.3) {
                flicker();
                done3 = true;
            }
        }
        else if (!(gamepad2.y) && done3 == true)
        {
            pressed3 = false;
            shooterWheel.setPower(0);
        }*/
        
        if(gamepad2.b && doorOpen) //move door to close position
        {
            indexRight.setPosition(.46);
            indexLeft.setPosition(.6);
            doorOpen=!doorOpen;
        }
        else if(gamepad2.b && !doorOpen)//move door to open position
        {
            indexRight.setPosition(1);
            indexLeft.setPosition(0);
            doorOpen=!doorOpen;
        }
        if(gamepad1.dpad_up)//move door up more
        {
            indexRight.setPosition(.38);
            indexLeft.setPosition(.68);
        }
        else if(gamepad1.dpad_down) //move door down a little
        {
            indexRight.setPosition(.57);
            indexLeft.setPosition(.47);
        }

        //shooter
        if(gamepad2.left_trigger>0)
        {
            shooterWheel.setPower(-.457);
            telemetry.addData("shooter power: ", shooterWheel.getPower());
            telemetry.update();
        }
        else if(gamepad2.right_trigger>0)
        {
            shooterWheel.setPower(-.45);
            telemetry.addData("shooter power: ", shooterWheel.getPower());
            telemetry.update();
        }
        else
        {
            shooterWheel.setPower(0);
        }

        if(gamepad2.a)
        {
            shooterFlicker.setPosition(.6);
        }
        else
        {
            shooterFlicker.setPosition(.3);
        }

        //Drivetrain
        float drive = gamepad1.left_stick_x;
        float strafe = gamepad1.left_stick_y;
        float turn = gamepad1.right_stick_x;

        if(gamepad1.a)
        {
            halfPower=!halfPower;
        }
        if (!halfPower)
        {
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

        //claw and pivot
        //drop over wall
        if (gamepad2.left_stick_y>0 && pressed1 == false)
        {
            done1 = false;
            if (!pressed1)
            {
                timer.reset();
                pressed1 = true;
            }
        }
        else if (pressed1 == true && done1 == false) {
            if (timer.seconds() > 0 && timer.seconds() < .5)
            {
                wobblePivotTop.setPosition(0.6);
                wobblePivotBottom.setPosition(0.4);
                wobbleThirdPivot.setPosition(0.6);
            }
            if (timer.seconds() > 1 && timer.seconds() < 1.5) {
                wobbleClaw.setPosition(.75);
            }
            if (timer.seconds() > 1.5 && timer.seconds() < 2) {
                wobbleClaw.setPosition(1);
                wobblePivotTop.setPosition(0);
                wobblePivotBottom.setPosition(1);
                wobbleThirdPivot.setPosition(0);
                done1 = true;
            }
        }
        else if (!(gamepad2.left_trigger > 0) && done1 == true)
        {
            pressed1 = false;
        }
        //grab wobble, pivot into robot
        if(gamepad2.left_stick_y<0 && pressed2 == false)
        {
            done2 = false;
            if(!pressed2)
            {
                timer.reset();
                pressed2 = true;
            }
        }
        else if (pressed2 == true && done2 == false)
        {
            if (timer.seconds() > 0 && timer.seconds() < .5)
            {
                wobbleClaw.setPosition(1);
            }
            if(timer.seconds() > .5)
            {
                wobblePivotTop.setPosition(0);
                wobblePivotBottom.setPosition(1);
                wobbleThirdPivot.setPosition(0);
                done2 = true;
            }
        }
        else if (!(gamepad2.left_trigger < 0) && done2 == true)
        {
            pressed2 = false;
        }

        //lead screw
        if(gamepad2.dpad_up)
        {
            wobbleLead.setPower(1);
        }
        else if(gamepad2.dpad_down)
        {
            wobbleLead.setPower(-1);
        }
        else
            wobbleLead.setPower(0);

        //just pivot
        if(gamepad2.right_stick_y>.1)
        {
            wobblePivotTop.setPosition(1);
            wobblePivotBottom.setPosition(0);
            wobbleThirdPivot.setPosition(1);
            wobbleClaw.setPosition(.75);
        }
        else if (gamepad2.right_stick_y<-.1)
        {
            wobbleClaw.setPosition(1);
            wobblePivotTop.setPosition(0);
            wobblePivotBottom.setPosition(1);
            wobbleThirdPivot.setPosition(0);
        }
        
    }
    
    public void flicker()
    {
        shooterFlicker.setPosition(.6);
        runtime.reset();
        while (runtime.seconds()<0.2)
        {
        }
        shooterFlicker.setPosition(.35);
    }
    public void turnTime(double time, double power)
    {
        runtime.reset();
        while(runtime.seconds()<time)
        {
            driveFrontRight.setPower(power);
            driveFrontLeft.setPower(-power);
            driveBackRight.setPower(power);
            driveBackLeft.setPower(-power);
        }
    }
    public void turn(double power)
    {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(-power);
        driveBackRight.setPower(power);
        driveBackLeft.setPower(-power);
    }
    public void crab(double power)
    {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(-power);
        driveBackRight.setPower(-power);
        driveBackLeft.setPower(power);
    }
    public void encoderCrab(int inches, double power){
        final double WHEEL_DIAMETER = 7.5; //in cm
        final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM*2.54*inches);

        while(Math.abs(driveBackRight.getCurrentPosition()) < Math.abs(STRAIGHT_COUNTS)){
            crab(power);
            telemetry.addData("STRAIGHT_COUNTS",STRAIGHT_COUNTS);
            telemetry.addData("POSITION", driveBackRight.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
        return;
    }
    public void stopMotors()
    {
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
    }
    public void resetEncoders()
    {
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders()
    {
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

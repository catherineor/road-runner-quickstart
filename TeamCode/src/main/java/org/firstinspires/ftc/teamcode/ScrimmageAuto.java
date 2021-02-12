// Marlbots-2020-2021-Auto-Code
// negative = turning clockwise
// positive = turning counter clockwise

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="Scrimmage Auto", group="Linear Opmode")

public class ScrimmageAuto extends LinearOpMode {
    public State state;
    private ElapsedTime runtime = new ElapsedTime();

    //Drivetrain
    private DcMotor driveFrontRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackRight;
    private DcMotor driveBackLeft;

    //shooter
    private DcMotor shooterWheel;
    private Servo shooterFlicker;

    //indexing
    private DcMotor index;
    private Servo indexRight;
    private Servo indexLeft;

    //intake motor
    private DcMotor intake;

    //wobble goal
    private Servo wobbleDolly;
    private Servo wobblePivotTop;
    private Servo wobblePivotBottom;
    private Servo wobbleClawTop;
    private Servo wobbleClawBottom;

    private double drive, strafe = 0;

    private int ringHeight;

    private enum State {
        TOSCAN,
        /*SCAN,
        SHOOTPS,
        TOTZ,
        DROPOFF,
        PARK,*/
        STOP,
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Drivetrain
        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");

        //indexing
        index = hardwareMap.get(DcMotor.class, "index");
        indexRight = hardwareMap.get(Servo.class, "indexRight");
        indexLeft = hardwareMap.get(Servo.class, "indexLeft");
        //open trapdoor on initalization

        //shooter
        shooterWheel = hardwareMap.get(DcMotor.class, "shooterWheel");
        shooterFlicker = hardwareMap.get(Servo.class, "shooterFlicker");
        //set shooter flicker position to not blocking

        //intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        //wobble goal
        wobbleDolly = hardwareMap.get(Servo.class, "wobbleDolly");
        wobblePivotTop = hardwareMap.get(Servo.class, "wobblePivotTop");
        wobblePivotBottom = hardwareMap.get(Servo.class, "wobblePivotBottom");
        wobbleClawTop = hardwareMap.get(Servo.class, "wobbleClawTop");
        wobbleClawBottom = hardwareMap.get(Servo.class, "wobbleClawBottom");
        //move pivot to down position, dolly to starting position, and make sure claws are open

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
        useEncoders();
        telemetry.addData("pos", driveFrontLeft.getCurrentPosition());
        telemetry.update();
        waitForStart();
        resetEncoders();
        useEncoders();
        state = State.TOSCAN;
        runtime.reset();
        while (opModeIsActive())
        {
            //correction = checkDirection();
            switch(state) {
                case TOSCAN:
                    resetEncoders();
                    useEncoders();
                    encoderForwards(20, .5);
                    setStateRunning(State.STOP);
                    break;
                case STOP:
                    stopMotors();
            }
    /*case SCAN:
    //add tensorflow/vuforia stuff
    break;
    case SHOOTPS:
      shooterWheel.setPower(0.5);
      shooterFlicker.setPosition(0.5);
      //set shooter wheel running while all 3 rings are shot
      setStateRunning(State.TOTZ);
    break;
    case TOTZ:
      resetEncoders();
      useEncoders();
      encoderForwards(20, .5);
      resetEncoders();
      useEncoders();
      encoderCrab(20, 5);//check direction
      setStateRunning(State.PARK);
    break;
    case DROPOFF:
    break;
    case PARK:
      resetEncoders();
      useEncoders();
      encoderBackwards(40, .5);
      setStateRunning(State.STOP);
    break;*/
        }
    }

    public void setStateRunning(State s)
    {
        resetEncoders();
        useEncoders();
        stopMotors();
        state = s;
        runtime.reset();
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

    public void encoderForwards(int inches, double power)
    {
        final double WHEEL_DIAMETER = 7.5; //in cm
        final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM*2.54*inches);

        if (driveBackRight.getCurrentPosition()) < STRAIGHT_COUNTS)
        {
            drive(power);
        }
    else
        stopMotors();
        telemetry.addDate("STRAIGHT_COUNTS",STRAIGHT_COUNTS)
    }

    public void encoderCrab(int inches, double power) {
        final double WHEEL_DIAMETER = 7.5; //in cm
        final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM * 2.54 * inches);

        if (Math.abs(driveBackRight.getCurrentPosition()) < Math.abs(STRAIGHT_COUNTS)) {
            crab(power);
        else
            stopMotors();

            telemetry.addData("STRAIGHT_COUNTS", STRAIGHT_COUNTS);
        }

        public void encoderBackwards ( int inches, double power){
            final double WHEEL_DIAMETER = 7.5; //in cm
            final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
            final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM * 2.54 * inches * -1);
            driveFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
            driveFrontLeft.setTargetPosition(STRAIGHT_COUNTS);
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive(-power);

            while (driveFrontLeft.isBusy()) {
                telemetry.addData("POS", driveFrontLeft.getCurrentPosition());
                telemetry.addData("STRAIGHT_COUNTS", STRAIGHT_COUNTS);
                telemetry.update();
            }
            stopMotors();

            telemetry.addData("POS", driveFrontLeft.getCurrentPosition());
            telemetry.addData("STRAIGHT_COUNTS", STRAIGHT_COUNTS);
            telemetry.update();
        }

        public void drive ( double power)
        {
            driveFrontRight.setPower(power);
            driveFrontLeft.setPower(power);
            driveBackRight.setPower(power);
            driveBackLeft.setPower(power);
        }

        public void crab (double power)
        {
            driveFrontRight.setPower(power);
            driveFrontLeft.setPower(-power);
            driveBackRight.setPower(-power);
            driveBackLeft.setPower(power);
        }

        public void turn (double power)
        {
            driveFrontRight.setPower(power);
            driveFrontLeft.setPower(-power);
            driveBackRight.setPower(power);
            driveBackLeft.setPower(-power);
        }

        public void stopMotors ()
        {
            driveFrontRight.setPower(0);
            driveFrontLeft.setPower(0);
            driveBackRight.setPower(0);
            driveBackLeft.setPower(0);
        }
    }

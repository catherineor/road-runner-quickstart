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

@Autonomous(name="Shoot Auto", group="Linear Opmode")

public class ShootAuto extends LinearOpMode {
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
    boolean changeFlicker = true;
    private float flickerPos = 0;

    //indexing
    private DcMotor index;
    private Servo indexRight;
    private Servo indexLeft;

    //intake motor
    private DcMotor intake;

    //wobble goal
    private DcMotor wobbleLead;
    private Servo wobblePivotTop;
    private Servo wobblePivotBottom;
    private Servo wobbleClaw;

    private double drive, strafe = 0;

    private int ringHeight;

    private enum State {
        TOLL,
        SHOOT1,
        SHOOT2,
        SHOOT3,
        PARK,
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
        shooterFlicker.setPosition(flickerPos);

        //intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        //wobble goal
        wobbleLead = hardwareMap.get(DcMotor.class, "wobbleLead");
        wobblePivotTop = hardwareMap.get(Servo.class, "wobblePivotTop");
        wobblePivotBottom = hardwareMap.get(Servo.class, "wobblePivotBottom");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");

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
        state = State.TOLL;
        runtime.reset();
        ElapsedTime autoTime = new ElapsedTime();
        while (opModeIsActive())
        {
            //correction = checkDirection();
            switch(state) {
                case TOLL:
                    shooterFlicker.setPosition(0);
                    shooterWheel.setPower(-.57);
                    resetEncoders();
                    useEncoders();
                    encoderCrab(5, .35);
                    resetEncoders();
                    useEncoders();
                    encoderForwards(54, .35);
                    //turn(17);
                    resetEncoders();
                    useEncoders();
                    encoderCrab(5, .35);
                    setStateRunning(State.SHOOT1);
                    break;
                case SHOOT1:
                    shooterWheel.setPower(-.57);
                    flicker();
                    setStateRunning(State.SHOOT2);
                    break;
                case SHOOT2:
                    shooterFlicker.setPosition(0);
                    shooterWheel.setPower(-.57);
                    encoderCrab(7, .35);
                    flicker();
                    setStateRunning(State.SHOOT3);
                case SHOOT3:
                    shooterFlicker.setPosition(0);
                    shooterWheel.setPower(-.57);
                    encoderCrab(5, .35);
                    flicker();
                    setStateRunning(State.PARK);
                case PARK:
                    resetEncoders();
                    useEncoders();
                    encoderForwards(5, .5);
                    setStateRunning(State.STOP);
                    break;
                case STOP:
                    stopMotors();
            }
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

    public void encoderForwards(int inches, double power){
        final double WHEEL_DIAMETER = (7.5/2.54);
        final double COUNTS_PER_INCH = 537.6 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_INCH*inches);

        while (Math.abs(driveBackRight.getCurrentPosition()) < Math.abs(STRAIGHT_COUNTS)){
            drive(power);
            telemetry.addData("STRAIGHT_COUNTS",STRAIGHT_COUNTS);
            telemetry.addData("POSITION", driveBackRight.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();

        return;
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

    public void encoderBackwards(int inches, double power){
        final double WHEEL_DIAMETER = 7.5; //in cm
        final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM*2.54*inches*-1);

        while(driveBackRight.getCurrentPosition() > STRAIGHT_COUNTS)
        {
            drive(-power);
            telemetry.addData("STRAIGHT_COUNTS", STRAIGHT_COUNTS);
            telemetry.addData("POSITION", driveBackRight.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();

        return;
    }

    public void drive(double power)
    {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(power);
        driveBackRight.setPower(power);
        driveBackLeft.setPower(power);
    }

    public void crab(double power)
    {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(-power);
        driveBackRight.setPower(-power);
        driveBackLeft.setPower(power);
    }

    public void turn(double power)
    {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(-power);
        driveBackRight.setPower(power);
        driveBackLeft.setPower(-power);
    }

    public void stopMotors()
    {
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
    }
    public void flicker()
    {
        shooterFlicker.setPosition(1);
        runtime.reset();
        while (runtime.seconds()<0.8)
        {
        }
        shooterFlicker.setPosition(0);
    }

    public void shooterOnly(float seconds,ElapsedTime time){
        while(time.seconds()>seconds){
            driveFrontRight.setPower(0);
            driveFrontLeft.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);
        }

    }
    public void encoderShoot(int revs, double power){
        final double WHEEL_DIAMETER = 9; //in cm
        final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
        //final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM*2.54*inches);

        while(Math.abs(shooterWheel.getCurrentPosition()) < Math.abs(revs)){
            shooterWheel.setPower(power);
            //telemetry.addData("STRAIGHT_COUNTS",STRAIGHT_COUNTS);
            telemetry.addData("POSITION", shooterWheel.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
        return;
    }
}

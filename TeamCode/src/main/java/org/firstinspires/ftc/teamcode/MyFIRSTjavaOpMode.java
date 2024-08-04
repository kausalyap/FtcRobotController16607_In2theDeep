package teamcode;

import static com.qualcomm.robotcore.util.Range.scale;
import static java.lang.Math.abs;

//import static TrcCommonLib.trclib.TrcUtil.scaleRange;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MyFIRSTjavaOpMode extends LinearOpMode {
    //private IMU imu;
    private DcMotor Frontleft;
    private DcMotor Frontright;
    private DcMotor Backleft;
    private DcMotor Backright;
    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;
    private Servo Test;
    double tgtPower = 0;
    double servotgtPower = 1;
    public void initMotors () {
        Test = hardwareMap.get(Servo.class, "Test");
        Test.setDirection(Servo.Direction.valueOf("FORWARD"));
        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        Frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        Frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        Backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        Backright.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("FL Motor direction", Frontleft.getDirection());
        telemetry.addData("FR Motor direction", Frontright.getDirection());
        telemetry.addData("BL Motor direction", Backleft.getDirection());
        telemetry.addData("BR Motor direction", Backright.getDirection());
        telemetry.update();
        //Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("FL Motor direction", Frontleft.getMode());
        telemetry.addData("FR Motor direction", Frontright.getMode());
        telemetry.addData("BL Motor direction", Backleft.getMode());
        telemetry.addData("BR Motor direction", Backright.getMode());
        telemetry.update();
    }
    public void motorFRRTPos (){
        int targetpos = 1500;
        double DCmotortgtPower = 1;
        Frontright.setPower(DCmotortgtPower);
        Frontright.setTargetPosition(targetpos);
        Frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void motorFLRTPos (){
        int targetpos = 1500;
        double DCmotortgtPower = 1;
        Frontleft.setPower(DCmotortgtPower);
        Frontleft.setTargetPosition(targetpos);
        Frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void motorBLRTPos (){
        int targetpos = 1500;
        double DCmotortgtPower = 1;
        Backleft.setPower(DCmotortgtPower);
        Backleft.setTargetPosition(targetpos);
        Backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void motorBRRTPos (){
        int targetpos = 1500;
        double DCmotortgtPower = 1;
        Backright.setPower(DCmotortgtPower);
        Backright.setTargetPosition(targetpos);
        Backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void motorgetParams (){
        telemetry.addData("FL Motor Power", Frontleft.getPower());
        telemetry.addData("FL Motor direction", Frontleft.getDirection());
        telemetry.addData("FL Motor Pos", Frontleft.getCurrentPosition());
        telemetry.addData("FR Motor Power", Frontright.getPower());
        telemetry.addData("FR Motor direction", Frontright.getDirection());
        telemetry.addData("FR Motor Pos", Frontright.getCurrentPosition());
        telemetry.addData("BL Motor Power", Backleft.getPower());
        telemetry.addData("BL Motor direction", Backleft.getDirection());
        telemetry.addData("BL Motor Pos", Backleft.getCurrentPosition());
        telemetry.addData("BR Motor Power", Backright.getPower());
        telemetry.addData("BR Motor direction", Backright.getDirection());
        telemetry.addData("BR Motor Pos", Backright.getCurrentPosition());
        telemetry.update();
    }
    public void mecanumdrive () {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        //comment 7/01/2024

        Frontleft.setPower(v1);
        Frontright.setPower(v2);
        Backleft.setPower(v3);
        Backright.setPower(v4);
    }
    public void holonomicdrive() {
        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.left_stick_x;
        double Strafe = gamepad1.right_stick_x;
        double MAX_SPEED = 1.0;
        //      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

        double Magnitude = abs(Speed) + abs(Turn) + abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        Frontleft.setPower(scale(((Speed) + (Turn) - (Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (Backleft != null) {
            Backleft.setPower(scale(((Speed) + (Turn) + (Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        Frontright.setPower(scale(((Speed) - (Turn) + (Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (Backright != null) {
            Backright.setPower(scale(((Speed) - (Turn) - (Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
    }
    public void allmotorsfwd () {
        tgtPower = this.gamepad1.left_stick_x;
        Frontleft.setPower(tgtPower);
        Frontright.setPower(tgtPower);
        Backright.setPower(tgtPower);
        Backleft.setPower(tgtPower);
        telemetry.addData("Target Power", tgtPower);
        motorgetParams ();
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
    @Override
    public void runOpMode () {
        //imu = hardwareMap.get(IMU.class, "imu");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        initMotors();
        waitForStart();

        while (opModeIsActive()) {
            //allmotorsfwd ();
            //holonomicdrive ();
            mecanumdrive ();
            //Test.getDirection();
            // check to see if we need to move the servo.
            if(gamepad1.y) {
                motorFLRTPos ();
                motorgetParams();
            } else if (gamepad1.x) {
                motorFRRTPos();
                motorgetParams();
            } else if (gamepad1.b) {
                motorBLRTPos();
                motorgetParams();
            } else if (gamepad1.a) {
                motorBRRTPos();
                motorgetParams();
            } else if (gamepad2.a) {
                Test.setPosition(0.5);
                telemetry.addData("Servo Position", Test.getPosition());
                telemetry.addData("Target Power", servotgtPower);
                telemetry.update();
            } else if (gamepad2.x) {
                Test.setPosition(0);
                telemetry.addData("Servo Position", Test.getPosition());
                telemetry.addData("Target Power", servotgtPower);
                telemetry.update();}
            //telemetry.addData("Motor Power", Test.getPower());
            Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Frontleft.setPower(0);
            Frontright.setPower(0);
            Backright.setPower(0);
            Backleft.setPower(0);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

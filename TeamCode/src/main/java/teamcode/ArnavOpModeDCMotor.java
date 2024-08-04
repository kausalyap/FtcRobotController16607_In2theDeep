package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArnavOpModeDCMotor extends LinearOpMode {
    //private Gyroscope imu;
    private DcMotor Backleft;
    private DcMotor Backright;
    private DcMotor Frontright;
    private DcMotor Frontleft;
    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;
    private Servo Test;

    static final double INCREMENT = 0.01;
    static final int    CYCLE_MS  =   50;
    static final double MAX_FWD   =  1.0;
    static final double MAX_REV   = -1.0;

    double power = 0;
    int cp = 0;
    boolean rampUp = true;

    public void init_motors() {
        Test = hardwareMap.get(Servo.class, "Test");
        Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFL", Frontleft.getDirection());
        Frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFR", Frontright.getDirection());
        Backright.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBR", Backright.getDirection());
        Backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBL", Backleft.getDirection());
        Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFL:", Frontleft.getCurrentPosition());
        Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFR:", Frontright.getCurrentPosition());
        Backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBL:", Backleft.getCurrentPosition());
        Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBR:", Backleft.getCurrentPosition());
        Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
    }
    public void run_to_position_FL() {
        int tgtPosition = 8000;
        cp = -1 * Frontleft.getCurrentPosition();
        Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontleft.setTargetPosition(tgtPosition);
        Frontleft.setPower(1);
        Frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Frontleft.isBusy()) {

            telemetry.addData("DcMotor PositionFL:", Frontleft.getCurrentPosition());
            telemetry.addData("cp", cp);
            telemetry.update();
        } Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontleft.setPower(0);
    }
    public void run_to_position_FR() {
        int tgtPosition = 8000;
        cp = -1 * Frontright.getCurrentPosition();
        Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontright.setTargetPosition(tgtPosition);
        Frontright.setPower(1);
        Frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Frontright.isBusy()) {

            telemetry.addData("DcMotor PositionFR:", Frontright.getCurrentPosition());
            telemetry.addData("cp", cp);
            telemetry.update();
        } Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontright.setPower(0);
    }
    public void run_to_position_BL() {
        int tgtPosition = 8000;
        cp = -1 * Backleft.getCurrentPosition();
        Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backleft.setTargetPosition(tgtPosition);
        Backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backleft.setPower(1);
        while (Backleft.isBusy()) {

            telemetry.addData("DcMotor PositionBL:", Backleft.getCurrentPosition());
            telemetry.addData("cp", cp);
            telemetry.update();
        } Backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backleft.setPower(0);
    }
    public void run_to_position_BR() {
        int tgtPosition = 8000;
        cp = -1 * Backright.getCurrentPosition();
        Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backright.setTargetPosition(tgtPosition);
        Backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backright.setPower(1);
        while (Backright.isBusy()) {

            telemetry.addData("DcMotor PositionBR:", Backright.getCurrentPosition());
            telemetry.addData("cp", cp);
            telemetry.update();
        } Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backright.setPower(0);
    }
    public void Mecanumdrive () {
        Frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        Backright.setDirection(DcMotorSimple.Direction.REVERSE);
        double h = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = h * Math.cos(robotAngle) + rightX;
        final double v2 = h * Math.sin(robotAngle) - rightX;
        final double v3 = h * Math.sin(robotAngle) + rightX;
        final double v4 = h * Math.cos(robotAngle) - rightX;

        Frontleft.setPower(v1);
        Frontright.setPower(v2);
        Backleft.setPower(v3);
        Backright.setPower(v4);
    }

    @Override
    public void runOpMode () {

        /*imu = hardwareMap.get(Gyroscope.class, "imu");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        */
        init_motors();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double tgtPower = 0;
        //test 78

        waitForStart();

        while (opModeIsActive()) {

           /* tgtPower = gamepad1.left_stick_y;
            Backleft.setPower(tgtPower);
            Backright.setPower(tgtPower);
            Frontleft.setPower(tgtPower);
            Frontright.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Status", "Motors");
            telemetry.update();
            */
            Mecanumdrive();
            if (gamepad1.a) {
                run_to_position_FL();
            } else if (gamepad1.b) {
                run_to_position_FR();
            } else if (gamepad1.x) {
                run_to_position_BL();
            } else if (gamepad1.y) {
                run_to_position_BR();
            } else if (gamepad2.a) {
                Test.setPosition(0);
            } else if (gamepad2.b) {
                Test.setPosition(0.25);
            } else if (gamepad2.x) {
                Test.setPosition(0.5);
            } else if (gamepad2.y) {
                Test.setPosition(1);
            }

        }

          /*  if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            if (gamepad1.x) {
                Test.setPosition(0);
            } else if (gamepad1.y || gamepad1.b) {
                Test.setPosition(0.5);
            } else if (gamepad1.a) {
                Test.setPosition(1);
            }

            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            tEst.setPower(power);
            sleep(CYCLE_MS);
            idle();
        }
            tEst.setPower(0);
            telemetry.addData(">", "Done");
            telemetry.update();
*/

        }
    }



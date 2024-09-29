package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import java.util.List;
import java.util.Locale;

@TeleOp
public class ArnavOpModeUpdatedWithConstants extends LinearOpMode {

    // Constants
    private static final int LADDER_POSITION_BOTTOM = 0;
    private static final int LADDER_POSITION_MIDDLE = 150;
    private static final int LADDER_POSITION_HIGH = 300;
    private static final int LADDER_POSITION_TOP = 600;

    private static final double LADDER_POWER_LOW = 0.1;
    private static final double LADDER_POWER_MEDIUM = 0.5;
    private static final double LADDER_POWER_HIGH = 0.8;

    private static final int HOOK_POSITION_RETRACTED = 40;
    private static final int HOOK_POSITION_EXTENDED = 8400;
    private static final double HOOK_POWER = 0.6;

    private static final double TEST_SERVO_LEFT_POSITION = 0.0;
    private static final double TEST_SERVO_RIGHT_POSITION = 1.0;
    private static final double ARM_SERVO_OPEN_POSITION = 0.0;
    private static final double ARM_SERVO_CLOSE_POSITION = 1.0;
    private static final double DRONE_SERVO_OPEN_POSITION = 0.0;
    private static final double DRONE_SERVO_CLOSE_POSITION = 1.0;

    // Motors and servos
    private DcMotor ladderLift, hook, frontRight, frontLeft, backLeft, backRight;
    private Servo testServo, droneServo, armServo;

    // Sensors
    private Limelight3A limelight;
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;

    // Timer
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors();
        initNavx();
        initLimelight();
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            runNavx();
            runLimelight();
            controlMecanumDrive();
            controlServos();
            controlLadder();
            controlHook()
            telemetry.update();
            idle();
        }
        limelight.stop();
    }

    // Initialization Methods
    public void initMotors() {
        //initialize servos
        testServo = hardwareMap.get(Servo.class, "Test");
        armServo = hardwareMap.get(Servo.class, "Servoarm");
        droneServo = hardwareMap.get(Servo.class, "Drone");

        //initialize motors
        backLeft = hardwareMap.get(DcMotor.class, "Backleft");
        backRight = hardwareMap.get(DcMotor.class, "Backright");
        frontLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "Frontright");

        //initialize hook motor
        hook = hardwareMap.get(DcMotor.class, "Hook");
        hook.setDirection(DcMotorSimple.Direction.REVERSE);
        hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize ladder motor
        ladderLift = hardwareMap.get(DcMotor.class, "LadderLift");
        ladderLift.setDirection(DcMotorSimple.Direction.REVERSE);
        ladderLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ladderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize mecanum drive motors
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFL", frontLeft.getDirection());
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFR", frontRight.getDirection());
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBR", backRight.getDirection());
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBL", backLeft.getDirection());
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
    }

    // Initialization LimeLight
    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // Initialization Navx
    private void initNavx() throws InterruptedException {
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        timer.reset();
        while (navxMicro.isCalibrating()) {
            telemetry.addData("Calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().add("Gyro Calibrated.");
        telemetry.clear();
        telemetry.update();
    }

    // Run LimeLight
    private void runLimelight() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s", status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    }

    // Run Navx
    private void runNavx() {
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", formatRate(rates.zRotationRate));
        telemetry.addLine()
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
    }

    // Control Mecaum Drive
    private void controlMecanumDrive() {
        Frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        Backright.setDirection(DcMotorSimple.Direction.REVERSE);
        double h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = h * Math.sin(robotAngle) - rightX;
        final double v2 = h * Math.cos(robotAngle) + rightX;
        final double v3 = h * Math.cos(robotAngle) - rightX;
        final double v4 = h * Math.sin(robotAngle) + rightX;

        Frontleft.setPower(v1);
        Frontright.setPower(v2);
        Backleft.setPower(v3);
        Backright.setPower(v4);
    }

    // Control Servos
    private void controlServos() {
        if (gamepad1.a) {
            testServo.setPosition(TEST_SERVO_LEFT_POSITION);
        } else if (gamepad1.b) {
            testServo.setPosition(TEST_SERVO_RIGHT_POSITION);
        }

        if (gamepad2.a) {
            armServo.setPosition(ARM_SERVO_OPEN_POSITION);
        } else if (gamepad2.b) {
            armServo.setPosition(ARM_SERVO_CLOSE_POSITION);
        }

        if (gamepad2.x) {
            droneServo.setPosition(DRONE_SERVO_OPEN_POSITION);
        } else if (gamepad2.y) {
            droneServo.setPosition(DRONE_SERVO_CLOSE_POSITION);
        }
    }

    // Control Ladder
    private void controlLadder() {
        if (gamepad2.dpad_up) {
            runMotorToPosition(ladderLift, LADDER_POSITION_TOP, LADDER_POWER_HIGH);
        } else if (gamepad2.dpad_right) {
            runMotorToPosition(ladderLift, LADDER_POSITION_HIGH, LADDER_POWER_HIGH);
        } else if (gamepad2.dpad_left) {
            runMotorToPosition(ladderLift, LADDER_POSITION_MIDDLE, LADDER_POWER_MEDIUM);
        } else if (gamepad2.dpad_down) {
            runMotorToPosition(ladderLift, LADDER_POSITION_BOTTOM, LADDER_POWER_LOW);
        }
    }

    // Control Hook
    private void controlHook() {
        if (gamepad1.left_bumper) {
            runMotorToPosition(hook, HOOK_POSITION_RETRACTED, HOOK_POWER);
        } else if (gamepad1.right_bumper) {
            runMotorToPosition(hook, HOOK_POSITION_EXTENDED, HOOK_POWER);
        }
    }

    //TODO: Review this method to see if the DcMotor motor, works as intended.
    private void runMotorToPosition(DcMotor motor, int targetPosition, double power) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        while (motor.isBusy() && opModeIsActive()) {
            telemetry.addData("Running to", targetPosition);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Helper Methods
    private String formatRate(float rate) {
        return String.format(Locale.getDefault(), "%.3f", rate);
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", degrees);
    }
}

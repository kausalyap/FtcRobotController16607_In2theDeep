package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ColtonOpModeDCMotor extends LinearOpMode {
    //private IMU imu;
    private DcMotor Frontleft;
    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;
    private Servo Test;

    @Override
    public void runOpMode () {
        //imu = hardwareMap.get(IMU.class, "imu");
        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        Test = hardwareMap.get(Servo.class, "Test");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double tgtPower = 0;
        double servotgtPower = 1;
        waitForStart();

        while (opModeIsActive()) {
            tgtPower = this.gamepad1.left_trigger;
            Frontleft.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", Frontleft.getPower());
            telemetry.addData("Motor direction", Frontleft.getDirection());
            telemetry.addData("Status", "Running");
            telemetry.update();
            //Test.getDirection();
            Test.setDirection(Servo.Direction.valueOf("FORWARD"));
            // check to see if we need to move the servo.
            if(gamepad1.y) {
                // move to 0 degrees.
                //Test.setPosition(0);
                Frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
                telemetry.addData("Motor direction", Frontleft.getDirection());
                telemetry.update();
                //Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Motor direction", Frontleft.getMode());
                telemetry.update();
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                Frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
                telemetry.addData("Motor direction", Frontleft.getDirection());
                telemetry.update();
                Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("Motor direction", Frontleft.getMode());
                telemetry.update();
                //Test.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                Test.setPosition(1);
                telemetry.addData("Motor direction", Frontleft.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("Servo Position", Test.getPosition());
            telemetry.addData("Target Power", servotgtPower);
            //telemetry.addData("Motor Power", Test.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

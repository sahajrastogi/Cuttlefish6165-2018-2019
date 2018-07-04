package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;


/**
 * Created by Lenovo on 9/30/2017.
 */

@TeleOp(name="TankBot")
public class TankBot extends OpMode {
    BNO055IMU imu;
    Orientation angles;
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorBR;

    double leftX, leftY, rightY;
    boolean isTank;
    boolean aControl;
    boolean isAnalog;
    boolean bControl;
    boolean halfSpeed;
    boolean yControl;
    boolean gyroRightInit;
    boolean gyroRightRunning;
    boolean gyroRightSpecialCase;

    boolean gyroLeftInit;
    boolean gyroLeftRunning;
    boolean gyroLeftSpecialCase;
    boolean rightControl;

    double rightTurnHeading;

    @Override
    public void init(){
        motorBR = hardwareMap.dcMotor.get("BR");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        gamepad1.setJoystickDeadzone((float)0.05);
        gamepad2.setJoystickDeadzone((float)0.05);

        isTank = true;
        isAnalog = false;
        halfSpeed = false;

        gyroLeftRunning = false;
        gyroLeftInit = false;
        gyroRightRunning = false;
        gyroRightInit = false;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void loop(){

        if(gamepad1.a){
            aControl = true;
        }
        if(!gamepad1.a && aControl){
            isTank = !isTank;
            aControl = false;
        }

        if(gamepad1.b){
            bControl = true;
        }

        if(!gamepad1.b && bControl){
            isAnalog = !isAnalog;
            bControl = false;
        }

        if(gamepad1.y){
            yControl = true;
        }

        if(!gamepad1.y && yControl){
            halfSpeed = !halfSpeed;
            yControl = false;
        }

        if(gamepad1.dpad_right){
            rightControl = true;
        }

        if(!gamepad1.dpad_right && rightControl){
            gyroRightInit = true;
            rightControl = false;
        }
        //glyph slides

        //joystick input
        if(!isAnalog) {
            leftY = Range.clip(scaleInput(gamepad1.left_stick_y), -0.95, 0.95);
            rightY = Range.clip(scaleInput(gamepad1.right_stick_y), -0.95, 0.95);
            leftX = Range.clip(scaleInput(-gamepad1.left_stick_x), -0.95, 0.95);
        } else {
            double lsy, rsy, lsx;
            if(gamepad1.left_stick_y > 0.1){
                lsy = 1;
            } else if(gamepad1.left_stick_y < -0.1){
                lsy = -1;
            } else {
                lsy = 0;
            }
            if(gamepad1.right_stick_y > 0.1){
                rsy = 1;
            } else if(gamepad1.right_stick_y  < -0.1){
                rsy = -1;
            } else {
                rsy = 0;
            }
            if(-gamepad1.left_stick_x > 0.1){
                lsx = 1;
            } else if(-gamepad1.left_stick_x  < -0.1){
                lsx = -1;
            } else {
                lsx = 0;
            }
            leftY = Range.clip(scaleInput( scaleInput(gamepad1.right_trigger) * lsy), -0.95, 0.95);
            rightY = Range.clip(scaleInput(scaleInput(gamepad1.right_trigger) * rsy), -0.95, 0.95);
            leftX = Range.clip(scaleInput(scaleInput(gamepad1.right_trigger) * lsx), -0.95, 0.95);
        }

        if(halfSpeed){
            leftX /= 2;
            rightY /= 2;
            leftY /= 2;
        }


        if(gyroRightInit){
            gyroRightRunning = true;
            gyroRightInit = false;
            // do init stuff
            rightTurnHeading = getHeading();
            if(rightTurnHeading >= -180 && rightTurnHeading <= -90){
                gyroRightSpecialCase = true;
            } else {
                gyroRightSpecialCase = false;
            }
            motorBR.setPower(0.1);
            motorFR.setPower(0.1);
            motorBL.setPower(-0.1);
            motorFL.setPower(-0.1);
        }

        if(gyroRightRunning){
            if(gyroRightSpecialCase){
                if(normalize(getHeading()) < normalize(rightTurnHeading) - 89){
                    motorBR.setPower(0);
                    motorFR.setPower(0);
                    motorBL.setPower(0);
                    motorFL.setPower(0);
                    gyroRightRunning = false;
                    gyroRightSpecialCase = false;
                }
            } else {
                if( getHeading() < rightTurnHeading-89){
                    motorBR.setPower(0);
                    motorFR.setPower(0);
                    motorBL.setPower(0);
                    motorFL.setPower(0);
                    gyroRightRunning = false;
                    gyroRightSpecialCase = false;
                }
            }


        }

        if(!gyroRightRunning) {
            // drive power
            if (isTank) {
                driveTank(leftY, rightY);
            } else {
                driveArcade(leftX, rightY);
            }
        }

        if(isTank) {
            telemetry.addData("Tank","");
        } else {
            telemetry.addData("Arcade","");
        }
        telemetry.addData("isAnalog",isAnalog);
        telemetry.addData("imu",getHeading());
        telemetry.addData("halfSpeed", halfSpeed);
        telemetry.addData("gyroRightSpecialCase", gyroRightSpecialCase);
        telemetry.addData("rightTurnHeading",rightTurnHeading);
        telemetry.update();
    }

    public void driveTank(double ly,  double ry){
        motorFL.setPower(ly);
        motorFR.setPower(ry);
        motorBL.setPower(ly);
        motorBR.setPower(ry);
    }
    public void driveArcade(double lx, double ry){
        motorFL.setPower(Range.clip(ry + lx,-1,1));
        motorFR.setPower(Range.clip(ry - lx,-1,1));
        motorBL.setPower(Range.clip(ry + lx,-1,1));
        motorBR.setPower(Range.clip(ry - lx,-1,1));
    }

    public void driveForward(double power){
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);

    }

    public double normalize(double hi){
        if(hi < 0){
            hi = -hi;
            hi = 180-hi;
            hi += 180;
        }
        return hi;
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * RED alliance first autonomous.
 * Uses the same intake power, gate positions and flywheel speed as BioBuzzRoboCentricRed.
 *
 * Sequence: gate closed -> intake on -> flywheel on -> wait -> gate open -> wait
 *           -> gate closed -> flywheel off -> drive forward -> intake off -> stop
 */
@Config
@Autonomous(name = "BB-Auto-Red-First", group = "Auto")
public class BBAutoRedFirst extends LinearOpMode {

    public static double SPINUP_WAIT_SEC = 3.0;   // flywheel on, gate closed
    public static double GATE_OPEN_SEC = 3.0;     // gate open, shooting
    public static double DRIVE_DISTANCE_IN = 20.0; // forward move after shooting

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double smoothedTargetRPM = 0.0;

    @Override
    public void runOpMode() {
        // Drive (Road Runner + Pinpoint)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Intake
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentricRed.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gate and shifter, same defaults as teleop (both closed)
        Servo gate = hardwareMap.get(Servo.class, BioBuzzRoboCentricRed.GATE_SERVO);
        Servo shifter = hardwareMap.get(Servo.class, BioBuzzRoboCentricRed.SHIFTER_SERVO);
        gate.setPosition(BioBuzzRoboCentricRed.CLOSED_POSITION);
        shifter.setPosition(BioBuzzRoboCentricRed.SHIFTER_CLOSED_POSITION);

        // Flywheels
        shooter1 = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentricRed.MOTOR_1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, BioBuzzRoboCentricRed.MOTOR_2_NAME);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(BioBuzzRoboCentricRed.MOTOR_2_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Ready - RED first auto");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double flywheelRPM = BioBuzzRoboCentricRed.FLYWHEEL_CLOSED_RPM;

        // Gate is closed. Start intake and flywheel, wait.
        intake.setPower(BioBuzzRoboCentricRed.INTAKE_POWER);
        holdFlywheel(flywheelRPM, SPINUP_WAIT_SEC, "Spinning up");

        // Open gate, wait.
        gate.setPosition(BioBuzzRoboCentricRed.OPEN_POSITION);
        holdFlywheel(flywheelRPM, GATE_OPEN_SEC, "Gate open");

        // Close gate, stop flywheel.
        gate.setPosition(BioBuzzRoboCentricRed.CLOSED_POSITION);
        smoothedTargetRPM = 0.0;
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);

        // Move forward.
        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DRIVE_DISTANCE_IN)
                            .build());
        }

        // Stop everything.
        intake.setPower(0.0);
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    /** Runs the flywheel at targetRPM for the given time, using the same control as teleop. */
    private void holdFlywheel(double targetRPM, double seconds, String label) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            double rpm1 = updateFlywheel(targetRPM);
            telemetry.addData("Step", label);
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.addData("Flywheel RPM", "target %.0f | measured %.0f", targetRPM, rpm1);
            telemetry.update();
            sleep(20);
        }
    }

    /** One control step, same slew limit and proportional correction as BioBuzzRoboCentricRed. */
    private double updateFlywheel(double targetRPM) {
        double maxRPMChangePerLoop = 60.0;
        smoothedTargetRPM += Range.clip(targetRPM - smoothedTargetRPM,
                -maxRPMChangePerLoop, maxRPMChangePerLoop);

        double actualRPM1 = (shooter1.getVelocity() * 60.0) / BioBuzzRoboCentricRed.TICKS_PER_REV;

        double power = 0.0;
        if (smoothedTargetRPM > 50.0) {
            double basePower = smoothedTargetRPM / 6000.0;
            double errorRPM = smoothedTargetRPM - actualRPM1;
            double kP = 0.00018;
            power = Range.clip(basePower + (errorRPM * kP), 0.0, 1.0);
        }
        shooter1.setPower(power);
        shooter2.setPower(power);
        return actualRPM1;
    }
}

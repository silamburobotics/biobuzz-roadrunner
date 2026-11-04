package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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

    public static double FLYWHEEL_RPM = 2800.0;   // auto shooter speed (teleop stays separate)
    public static double SPINUP_WAIT_SEC = 3.0;  // flywheel on, gate closed
    public static double GATE_OPEN_SEC = 3.0;     // gate open, shooting
    public static double DRIVE_DISTANCE_IN = 40.0; // forward move after shooting, then back the same distance
    public static double WAIT_AFTER_DRIVE_SEC = 3.0; // pause at the far point before driving back
    public static double WAIT_AFTER_GATE_OPEN_SEC = 3.0; // pause with the gate open back at the start line
    public static double STRAFE_RIGHT_IN = 40.0;  // sideways move to the right at the end
    public static double TURN_CLOCKWISE_DEG = 90.0; // clockwise turn after the strafe
    public static double MOVE_AFTER_TURN_IN = 40.0; // forward move (robot's front) after the turn

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

        double flywheelRPM = FLYWHEEL_RPM;

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

        // Wait, then drive back to the start line. The intake keeps running.
        ElapsedTime waitTimer = new ElapsedTime();
        while (opModeIsActive() && waitTimer.seconds() < WAIT_AFTER_DRIVE_SEC) {
            telemetry.addData("Step", "Waiting at far point");
            telemetry.update();
            sleep(20);
        }

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToX(0)
                            .build());
        }

        // Back at the start line: open the gate.
        if (opModeIsActive()) {
            gate.setPosition(BioBuzzRoboCentricRed.OPEN_POSITION);
        }

        // Wait with the gate open.
        ElapsedTime gateTimer = new ElapsedTime();
        while (opModeIsActive() && gateTimer.seconds() < WAIT_AFTER_GATE_OPEN_SEC) {
            telemetry.addData("Step", "Gate open at start line");
            telemetry.update();
            sleep(20);
        }

        // Strafe right. In Road Runner +Y is left, so right is -Y.
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            Actions.runBlocking(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(here.position.x, here.position.y - STRAFE_RIGHT_IN))
                            .build());
        }

        // Turn clockwise. In Road Runner counterclockwise is positive, so clockwise is negative.
        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(-Math.toRadians(TURN_CLOCKWISE_DEG))
                            .build());
        }

        // Move forward along the robot's current heading (after the turn this is not field +X).
        if (opModeIsActive()) {
            Pose2d here = drive.localizer.getPose();
            double heading = here.heading.toDouble();
            Actions.runBlocking(
                    drive.actionBuilder(here)
                            .strafeTo(new Vector2d(
                                    here.position.x + MOVE_AFTER_TURN_IN * Math.cos(heading),
                                    here.position.y + MOVE_AFTER_TURN_IN * Math.sin(heading)))
                            .build());
        }

        // Stop the flywheels. The intake is left running.
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);

        telemetry.addData("Status", "Done - intake running");
        telemetry.update();

        // Returning from runOpMode ends the OpMode and the SDK powers every motor off,
        // so stay here to keep the intake running until the auto period ends.
        while (opModeIsActive()) {
            sleep(20);
        }
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

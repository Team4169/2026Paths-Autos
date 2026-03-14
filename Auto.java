package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto {

    // ── Tuning constants ──────────────────────────────────────────────────
    private static final double DRIVE_SPEED_MPS = -1.0; // m/s backward — TUNE ME
    private static final double FOLD_TIME       = 0.4;
    private static final double DRIVE_TIME      = 4.2;  // TUNE ME
    private static final double SHOOT_TIME      = 7.0;

    // ── Auto phases ───────────────────────────────────────────────────────
    private enum Phase { FOLDING_DOWN, DRIVING, SHOOTING, DONE }
    private Phase m_phase;
    private final Timer m_timer = new Timer();

    // ── Drivetrain ────────────────────────────────────────────────────────
    private final SwerveSubsystem m_swerve;

    // ── Mechanisms ────────────────────────────────────────────────────────
    private final SparkMax  m_intakeMotor;
    private final SparkMax  m_intakeFoldMotor;
    private final SparkFlex m_shooterMotor;
    private final SparkFlex m_kickerMainMotor;
    private final SparkMax  m_kickerAuxMotor;

    public Auto(
            SwerveSubsystem swerve,
            SparkMax intakeMotor,
            SparkMax intakeFoldMotor,
            SparkFlex shooterMotor,
            SparkFlex kickerMainMotor,
            SparkMax kickerAuxMotor) {

        m_swerve          = swerve;
        m_intakeMotor     = intakeMotor;
        m_intakeFoldMotor = intakeFoldMotor;
        m_shooterMotor    = shooterMotor;
        m_kickerMainMotor = kickerMainMotor;
        m_kickerAuxMotor  = kickerAuxMotor;
    }

    /** Call once from Robot.autonomousInit() */
    public void init() {
        m_phase = Phase.FOLDING_DOWN;
        m_timer.reset();
        m_timer.start();
        stopAll();
        m_intakeFoldMotor.set(0.1);
    }

    /** Call every loop from Robot.autonomousPeriodic() */
    public void update() {
        switch (m_phase) {

            case FOLDING_DOWN:
                m_intakeFoldMotor.set(0.1);
                if (m_timer.hasElapsed(FOLD_TIME)) {
                    m_intakeFoldMotor.stopMotor();
                    m_timer.reset();
                    m_phase = Phase.DRIVING;
                    driveStraight();
                }
                break;

            case DRIVING:
                driveStraight();
                if (m_timer.hasElapsed(DRIVE_TIME)) {
                    m_swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
                    m_timer.reset();
                    m_phase = Phase.SHOOTING;
                    m_shooterMotor.set(-0.4);
                    m_kickerMainMotor.set(0.5);
                    m_kickerAuxMotor.set(-0.44);
                    m_intakeMotor.set(-0.8);
                }
                break;

            case SHOOTING:
                if (m_timer.hasElapsed(SHOOT_TIME)) {
                    stopAll();
                    m_phase = Phase.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    /** Call from Robot.autonomousExit() or if auto is interrupted. */
    public void stop() {
        stopAll();
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    private void driveStraight() {
        m_swerve.driveRobotRelative(new ChassisSpeeds(DRIVE_SPEED_MPS, 0, 0));
    }

    private void stopAll() {
        m_swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        m_intakeMotor.stopMotor();
        m_intakeFoldMotor.stopMotor();
        m_shooterMotor.stopMotor();
        m_kickerMainMotor.stopMotor();
        m_kickerAuxMotor.stopMotor();
    }
}
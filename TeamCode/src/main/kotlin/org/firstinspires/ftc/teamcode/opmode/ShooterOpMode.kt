package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.shooter.Shooter

@TeleOp(group = "Systems")
class ShooterOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var shooter: Shooter
    var limelight: Limelight3A? = null
    var shooterJob: Job? = null

    override fun init() {
        shooter = opModeGraph.shooter
        telemetry = opModeGraph.telemetry
        shooter.hood = 0.5
        telemetry = opModeGraph.telemetry
        limelight = opModeGraph.limelight
    }

    override fun start() {
        shooter.stateFlow
            .onEach {
                RobotLog.dd(ShooterOpMode::class.simpleName, "Shooter State: $it")
            }
            .launchIn(opModeGraph.opModeScope)
        limelight?.start()
    }

    override fun loop() {
        if (gamepad1.aWasPressed() && shooterJob == null) {
            shooterJob = shooter.shoot()
        }
        if (gamepad1.bWasPressed()) {
            shooterJob?.cancel()
            shooterJob = null
        }
        if (gamepad1.y) shooter.velocity += 1.0
        if (gamepad1.x) shooter.velocity -= 1.0
        if (gamepad1.dpad_up) {
            shooter.hood += 0.01
        }
        if (gamepad1.dpad_down) {
            shooter.hood -= 0.01
        }
        when {
            gamepad1.dpad_right -> shooter.angleDegrees -= 1.0
            gamepad1.dpad_left -> shooter.angleDegrees += 1.0
        }
        if (gamepad1.triangleWasPressed()) {
            limelight?.latestResult?.fiducialResults?.singleOrNull()?.targetXDegrees?.let { shooter.angleDegrees -= it }
        }
        telemetry.addData("Angle", shooter.angleDegrees)
    }
}

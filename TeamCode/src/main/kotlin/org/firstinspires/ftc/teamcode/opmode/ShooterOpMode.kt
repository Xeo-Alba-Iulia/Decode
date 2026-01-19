package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.shooter.Shooter

@TeleOp(group = "Systems")
open class ShooterOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var shooter: Shooter
    var shooterJob: Job? = null

    override fun init() {
        shooter = opModeGraph.shooter
        telemetry = opModeGraph.telemetry
        shooter.hood = 0.5
        telemetry = opModeGraph.telemetry
    }

    override fun start() {
        shooter.stateFlow
            .onEach {
                telemetry.addData("Shooter State", it)
            }
            .filter { (_, _, canShoot) -> canShoot }
            .onEach {
                RobotLog.dd(ShooterOpMode::class.simpleName, "Shooter State: $it")
            }
            .launchIn(opModeGraph.opModeScope)
    }

    override fun loop() {
        if (gamepad1.aWasPressed()) {
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
            gamepad1.dpad_right -> shooter.angleDegrees += 0.01
            gamepad1.dpad_left -> shooter.angleDegrees -= 0.01
        }
        telemetry.addData("Angle", shooter.angleDegrees)
    }
}

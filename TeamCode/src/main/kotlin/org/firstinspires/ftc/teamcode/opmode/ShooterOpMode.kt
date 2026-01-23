package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.shooter.ShooterImpl
import org.firstinspires.ftc.teamcode.sorter.Transfer

@TeleOp(group = "Systems")
class ShooterOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var shooter: ShooterImpl
    lateinit var transfer: Transfer
    var limelight: Limelight3A? = null
    var shooterJob: Job? = null

    override fun init() {
        shooter = opModeGraph.shooter as ShooterImpl
        telemetry = opModeGraph.telemetry
        transfer = opModeGraph.transfer
        shooter.hood = 0.5
        telemetry = opModeGraph.telemetry
        limelight = opModeGraph.limelight.also {
            it.pipelineSwitch(1)
        }
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
        val diff = gamepad1.right_trigger - gamepad1.left_trigger
        if (diff != 0f)
            shooter.power += diff * 0.001
        if (gamepad1.left_stick_y != 0f)
            shooter.hood += gamepad1.left_stick_y * (-0.001)
        when {
            gamepad1.dpad_right -> shooter.angleDegrees -= 1.0
            gamepad1.dpad_left -> shooter.angleDegrees += 1.0
        }
        if (gamepad1.triangleWasPressed()) {
            limelight?.latestResult?.fiducialResults?.singleOrNull()?.let {
                shooter.angleDegrees -= it.targetXDegrees
                val distance = it.targetPoseCameraSpace.position.z
                telemetry.addData("Distance", distance)
            }
        }
        when {
            gamepad1.squareWasPressed() -> transfer.isRunning = true
            gamepad1.squareWasReleased() -> transfer.isRunning = false
        }
        telemetry.addData("Hood", shooter.hood)
        telemetry.addData("Power", shooter.power)
    }

    override fun stop() {
        super.stop()
        limelight?.stop()
    }
}

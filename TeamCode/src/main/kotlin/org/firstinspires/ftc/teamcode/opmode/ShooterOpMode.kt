package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Job
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.shooter.ShooterImpl
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.sorter.Transfer

@TeleOp(group = "Systems")
class ShooterOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var shooter: ShooterImpl
    lateinit var intake: Intake
    lateinit var sorter: Sorter
    lateinit var transfer: Transfer
    var limelight: Limelight3A? = null
    var shooterJob: Job? = null

    override fun init() {
        intake = opModeGraph.intake
        sorter = opModeGraph.sorter.apply { prepareIntake() }
        shooter = opModeGraph.shooter as ShooterImpl
        telemetry = opModeGraph.telemetry
        transfer = opModeGraph.transfer
        shooter.hood = 0.5
        telemetry = opModeGraph.telemetry
        limelight = opModeGraph.limelight.apply { pipelineSwitch(1) }
    }

    override fun start() {
        limelight?.start()
        shooter.velocityOffset = 2200.0
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
            shooter.velocityOffset += diff
        if (gamepad1.left_stick_y != 0f)
            shooter.hood += gamepad1.left_stick_y * (-0.001)
        when {
            gamepad1.dpad_right -> shooter.angleDegrees -= 1.0
            gamepad1.dpad_left -> shooter.angleDegrees += 1.0
        }
        limelight?.latestResult?.fiducialResults?.singleOrNull()?.let {
            val distance = it.targetPoseCameraSpace.position.z
            telemetry.addData("Distance", distance)
            if (gamepad1.triangleWasPressed())
                shooter.angleDegrees -= it.targetXDegrees
        }

        when {
            gamepad1.squareWasPressed() -> transfer.isRunning = true
            gamepad1.squareWasReleased() -> transfer.isRunning = false
        }
        if (gamepad1.rightBumperWasPressed())
            intake.isRunning = !intake.isRunning

        when {
            gamepad1.dpadUpWasPressed() -> {
                sorter.intake(ArtefactType.PURPLE)
                if (sorter.isFull)
                    sorter.prepareShoot()
            }

            gamepad1.dpadDownWasPressed() -> {
                if (sorter.isEmpty)
                    sorter.prepareIntake()
                else
                    sorter.prepareShoot()
            }
        }

        telemetry.addData("Hood", shooter.hood)
        telemetry.addData("Speed", shooter.velocityOffset)
        telemetry.addData("Actual speed", shooter.stateFlow.value.velocity)
    }

    override fun stop() {
        super.stop()
        limelight?.stop()
    }
}

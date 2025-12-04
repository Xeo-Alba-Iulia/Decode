package org.firstinspires.ftc.teamcode.shooter

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.concurrent.atomics.AtomicReference
import kotlin.concurrent.atomics.ExperimentalAtomicApi
import kotlin.time.Duration.Companion.seconds

@SingleIn(OpModeScope::class)
@ContributesBinding(OpModeScope::class)
@OptIn(ExperimentalAtomicApi::class)
class ShooterNoEncoder(
    @Named("shooterMotor") private val motor: DcMotorEx,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
) : Shooter {
    private val runningJob = AtomicReference<Job?>(null)

    override var angleDegrees: Double by rotationServo::position
    override var hood by hoodServo::position
    override var velocity = 0.88
        set(value) {
            field = value.coerceIn(0.0, 1.0)
            if (isRunning) {
                motor.power = field
            }
        }
    override var isRunning = false
        private set

    override val isAtTarget = MutableStateFlow(false)

    override suspend fun turnOn() {
        motor.power = velocity
        if (isRunning) return
        val newJob = coroutineScope {
            launch {
                delay(2.seconds)
                isRunning = true
            }
        }
        runningJob.compareAndSet(null, newJob)
        newJob.join()
    }

    override fun turnOff() {
        motor.power = 0.0
        isRunning = false
        runningJob.exchange(null)?.cancel("Shooter turned off prematurely")
    }

    override fun toString() =
        "ShooterNoEncoder(angleDegrees=$angleDegrees, hood=$hood, velocity=$velocity, isRunning=$isRunning)"
}
package org.firstinspires.ftc.teamcode.shooter

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancel
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
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
    private var runningJob = AtomicReference<Job?>(null)

    override var angleDegrees: Double
        get() = (rotationServo.position * 120.0) - 60.0
        set(value) {
            rotationServo.position = (value + 60.0) / 120.0
        }
    override var hood by hoodServo::position
    override var shooterSpeed = 0.7
    override var isRunning = false
        private set

    override suspend fun turnOn() {
        if (isRunning) return
        motor.power = shooterSpeed
        runningJob.compareAndSet(null, coroutineScope {
            launch {
                delay(2.seconds)
                isRunning = true
            }
        })
    }

    override fun turnOff() {
        motor.power = 0.0
        isRunning = false
        runningJob.exchange(null)?.cancel("Shooter turned off prematurely")
    }

    override fun toString() =
        "ShooterNoEncoder(angleDegrees=$angleDegrees, hood=$hood, shooterSpeed=$shooterSpeed)"
}
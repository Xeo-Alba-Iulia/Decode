package org.firstinspires.ftc.teamcode.shooter

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlin.math.absoluteValue

class ShooterImpl(
    @Named("shooterMotor") private val motor: DcMotorEx,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
    opModeScope: CoroutineScope,
) : Shooter {

    private val _isAtTarget = MutableStateFlow(false)
    override val isAtTarget = _isAtTarget.asStateFlow()

    init {
        opModeScope.launch {
            while (true) {
                _isAtTarget.value = isRunning && (motor.velocity - velocity).absoluteValue < 20.0
                delay(50)
            }
        }
    }

    override var angleDegrees by rotationServo::position
    override var hood by hoodServo::position
    override var velocity = 0.0
        set(value) {
            field = value
            if (isRunning) {
                motor.velocity = field
            }
        }

    override var isRunning = false
        private set

    override suspend fun turnOn() {
        isRunning = true
        motor.velocity = velocity
    }

    override fun turnOff() {
        isRunning = false
        motor.velocity = 0.0
    }
}

package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.teamcode.InterpLUT
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.PI
import kotlin.math.abs

@Config
@ContributesBinding(OpModeScope::class)
class ShooterImpl(
    @Named("shooter") private val motor: DcMotorEx,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("turret") private val turretServos: List<Servo>,
    private val opModeScope: CoroutineScope,
) : Shooter {

    companion object {
        @JvmField
        var coefficients = PIDCoefficients(0.008, kD = 0.00035)
        @JvmField
        var parameters = BasicFeedforwardParameters(kS = 0.09, kV = 0.000315)

        const val TICKS_PER_SEC_TO_METERS_PER_SEC = 0.072 * PI / 28
        const val TURRET_ROTATION_PER_DEGREE = (.794 - .2025) / 180
        const val MAX_TURRET_ANGLE = 90.0
        const val TAG = "ShooterImpl"
    }

    val distances = listOf(0.86, 0.92, 1.4, 1.67, 1.97, 2.3, 3.01, 3.22)
    val velocityLUT: InterpLUT = InterpLUT(
        /* input = */ distances,
        /* output = */ listOf(1460.0, 1580.0, 1680.0, 1780.0, 1850.0, 1980.0, 2580.0, 2620.0),
        /* safeMode = */ true
    ).createLUT()

    val hoodLUT: InterpLUT = InterpLUT(
        distances,
        listOf(0.0, 0.0, 0.05, 0.05, 0.05, 0.12, 0.48, 0.45).map { 1.0 - it },
        true
    ).createLUT()

    private var _angleDegrees = Double.NaN
        set(value) {
            field = value
            turretServos.forEach { it.position = .5 - field * TURRET_ROTATION_PER_DEGREE }
        }
    override var angleDegrees = 0.0
        set(value) {
            field = value.coerceIn(-MAX_TURRET_ANGLE, MAX_TURRET_ANGLE)
            if (abs(field - _angleDegrees) >= 0.5 || _angleDegrees.isNaN()) {
                _angleDegrees = field
            }
        }

    var hood by hoodServo::position

    private val controller = controlSystem {
        velPid(coefficients)
        basicFF(parameters)
    }

    @Suppress("RedundantModalityModifier")
    final override val stateFlow: StateFlow<Shooter.State>
        field = MutableStateFlow(Shooter.State(0.0, false))

    private fun setPower(power: Double) {
        motor.power = power
    }

    private fun update(dist: Double = 0.0) {
        val velocity = motor.velocity
        if (dist > 2.8) {
            setPower(1.0)
            stateFlow.value = Shooter.State(velocity, false)
            Log.v(TAG, "Velocity: $velocity")
            return
        }
        val desiredVelocity = controller.goal.velocity
        Log.v(TAG, "Velocity: $velocity, Desired: $desiredVelocity")
        setPower(controller.calculate(KineticState(velocity = velocity)))
        stateFlow.value = Shooter.State(velocity, abs(velocity - desiredVelocity) <= 80.0)
    }

    override fun shoot(distanceFlow: Flow<Double>): Job =
        opModeScope.launch {
            try {
                var dist = 0.0
                distanceFlow.onEach { distance ->
                    hood = hoodLUT[distance]
                    controller.goal = KineticState(velocity = velocityLUT[distance])
                    dist = distance
                }.launchIn(this + Dispatchers.IO)
                while (true) {
                    update(dist)
                    delay(50L)
                }
            } finally {
                setPower(0.0)
                stateFlow.value = Shooter.State(0.0, false)
            }
        }

    fun shoot(velocityFn: () -> Double, hoodFn: () -> Double): Job =
        opModeScope.launch {
            try {
                while (true) {
                    controller.goal = KineticState(velocity = velocityFn())
                    hood = hoodFn()
                    update()
                    delay(50L)
                }
            } finally {
                setPower(0.0)
                stateFlow.value = Shooter.State(0.0, false)
            }
        }
}


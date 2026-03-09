package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.control.filters.LowPassFilter
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.teamcode.InterpLUT
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

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
        var coefficients = PIDCoefficients(0.005)
        @JvmField
        var parameters = BasicFeedforwardParameters(kS = 0.09, kV = 0.000345)

        const val TICKS_PER_SEC_TO_METERS_PER_SEC = 0.082 / 28
        const val TURRET_ROTATION_PER_DEGREE = (.794 - .2025) / 180
        const val MAX_TURRET_ANGLE = 90.0
    }

    val distances = listOf(0.86, 0.92, 1.4, 1.67, 1.97, 2.3, 3.01, 3.42)
    val velocityLUT: InterpLUT = InterpLUT(
        /* input = */ distances,
        /* output = */ listOf(1460.0, 1580.0, 1680.0, 1780.0, 1850.0, 1980.0, 2180.0, 2370.0),
        /* safeMode = */ true
    ).createLUT()

    val hoodLUT: InterpLUT = InterpLUT(
        listOf(25.5, 26.6, 29.0, 30.6, 32.3, 34.5, 36.8, 39.1, 42.0, 44.3, 46.5),
        (0..10).map { it / 10.0 },
        true
    ).createLUT()

    private var _angleDegrees = 0.0
        set(value) {
            field = value
            turretServos.forEach { it.position = .5 - field * TURRET_ROTATION_PER_DEGREE }
        }
    override var angleDegrees = 0.0
        set(value) {
            field = value.coerceIn(-MAX_TURRET_ANGLE, MAX_TURRET_ANGLE)
            if (abs(field - _angleDegrees) >= 0.5) {
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

    /**
     * Finds the launch angle for a given distance and velocity using Newton's method.
     *
     * @param distance The horizontal distance to the target in meters.
     * @param velocity The launch velocity in meters per second.
     * @param guess An initial guess for the launch angle in radians.
     *
     * @return The launch angle in radians that will hit the target at the given distance and velocity.
     */
    private tailrec fun findLaunchAngle(
        distance: Double,
        velocity: Double,
        guess: Double = Math.toRadians(31.0),
        repetitions: Int = 4
    ): Double? {
        val g = 8.95
        val height = 0.63
        val d = distance
        val v = velocity
        val sin = sin(guess)
        val cos = cos(guess)
        val f = d * sin / cos - (d * d * g) / (2 * v * v * cos * cos) - height
        return if (repetitions == 0) {
            val validatedGuess = guess.takeIf { f >= -0.1 }
            if (BuildConfig.DEBUG) {
                val guessDeg = validatedGuess?.let { Math.toDegrees(it) }
                Log.v("Angle", "Found: $guessDeg, distance = $distance, velocity = $velocity, f = $f")
            }
            validatedGuess
        } else {
            val df = d / (cos * cos) - (d * d * g * sin) / (v * v * cos * cos * cos)
            findLaunchAngle(distance, velocity, guess - f / df, repetitions - 1)
        }
    }

    private val hoodFilter = LowPassFilter(0.6, 45.0)

    var isUpdatingHood = true

    private fun update(distance: Double) {
        val position = motor.currentPosition.toDouble()
        val velocity = motor.velocity
        val desiredVelocity = controller.goal.velocity
        Log.v("ShooterImpl", "Velocity: $velocity, Desired: $desiredVelocity")
        setPower(controller.calculate(KineticState(position, velocity)))
        val isFar = distance > 2.3
        val isControllingServo = distance != 0.0 && velocity != 0.0 && isUpdatingHood
        stateFlow.value =
            Shooter.State(
                velocity,
                isControllingServo && findLaunchAngle(
                    distance, (if (isFar) velocity else desiredVelocity - 50.0) * TICKS_PER_SEC_TO_METERS_PER_SEC
                )?.let { result ->
                    hood = hoodFilter.filter(hoodLUT[Math.toDegrees(result)])
                    isFar || abs(velocity - desiredVelocity) <= 80.0
                } ?: false
            )
    }

    override fun shoot(distanceFlow: Flow<Double>): Job =
        opModeScope.launch {
            try {
                var dist = 0.0
                distanceFlow.onEach { distance ->
                    dist = distance
                    controller.goal = KineticState(velocity = velocityLUT[distance])
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
                    update(0.0)
                    delay(50L)
                }
            } finally {
                setPower(0.0)
                stateFlow.value = Shooter.State(0.0, false)
            }
        }
}


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
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@Config
@ContributesBinding(OpModeScope::class)
class ShooterImpl(
    @Named("shooter") private val motors: List<DcMotorEx>,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
    @Named("shooterEncoder") private val encoder: DcMotorEx,
    private val opModeScope: CoroutineScope,
) : Shooter {

    companion object {
        @JvmField
        var coefficients = PIDCoefficients(0.001)

        @JvmField
        var parameters = BasicFeedforwardParameters(kS = 0.05, kV = 0.00033)
    }

    /*
       0.92: 1420 0.0
       1.40: 1500 0.053
       1.67: 1800 0.41
       1.97: 1750 0.35
       2.3:  1750 0.31
       3.01: 2010 0.35
       3.42: 2040 0.332
     */
    val distances = listOf(0.92, 1.4, 1.67, 1.97, 2.3, 3.01, 3.42)
    val velocityLUT: InterpLUT = InterpLUT(
        /* input = */ distances,
        /* output = */ listOf(1420.0, 1500.0, 1760.0, 1800.0, 1860.0, 2020.0, 2150.0),
        /* safeMode = */ true
    ).createLUT()

    override var angleDegrees = 0.0
        set(value) {
            field = value.coerceIn(-80.0..80.0)
            rotationServo.position = 0.5 - field / 160.0
        }

    private var hood by hoodServo::position

    private val controller = controlSystem {
        velPid(coefficients)
        basicFF(parameters)
    }

    @Suppress("RedundantModalityModifier")
    final override val stateFlow: StateFlow<Shooter.State>
        field = MutableStateFlow(Shooter.State(0.0, false))

    private fun setPower(power: Double) = motors.forEach { it.power = power }

    /**
     * Finds the launch angle for a given distance and velocity using Newton's method.
     *
     * @param distance The horizontal distance to the target in meters.
     * @param velocity The launch velocity in meters per second.
     * @param guess An initial guess for the launch angle in degrees.
     *
     * @return The launch angle in degrees that will hit the target at the given distance and velocity.
     */
    private tailrec fun findLaunchAngle(
        distance: Double,
        velocity: Double,
        guess: Double = Math.toRadians(67.5),
        repetitions: Int = 2
    ): Double {
        val g = 8.9
        val height = 0.68
        if (velocity == 0.0 || distance == 0.0)
            return Math.toRadians(60.0)
        if (repetitions <= 0) {
            Log.v("Angle", "Found: $guess, distance = $distance, velocity = $velocity")
            return guess
        }
        val d = distance
        val v = velocity
        val sin = sin(guess)
        val cos = cos(guess)
        val f = d * sin / cos - (d * d * g) / (2 * v * v * cos * cos) - height
        val df = d / (cos * cos) - (d * d * g * sin) / (v * v * cos * cos * cos)
        return findLaunchAngle(distance, velocity, guess - f / df, repetitions - 1)
    }

    private fun update(distance: Double) {
        val position = encoder.currentPosition.toDouble()
        val velocity = encoder.velocity
        val desiredVelocity = controller.goal.velocity
        Log.v("ShooterImpl", "Velocity: $velocity, Desired: $desiredVelocity")
        setPower(controller.calculate(KineticState(position, velocity)))
        stateFlow.value = Shooter.State(velocity, abs(velocity - desiredVelocity) <= 80.0)
        val result = Math.toDegrees(findLaunchAngle(distance, ((velocity) / 28) * .08)).coerceIn(28.0..45.0)
        hood = (90 - result - 28) / (45 - 30)
    }

    override fun shoot(distanceFlow: Flow<Double>): Job =
        opModeScope.launch {
            try {
                var dist = 0.0
                distanceFlow.onEach { distance ->
                    dist = distance
                    controller.goal = KineticState(velocity = velocityLUT[distance])
                }.launchIn(this + Dispatchers.IO)
                withContext(Dispatchers.Default) {
                    while (true) {
                        update(dist)
                        delay(100L)
                    }
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


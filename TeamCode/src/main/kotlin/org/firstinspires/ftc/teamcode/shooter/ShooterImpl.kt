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
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.flowOn
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
        /* output = */ listOf(1420.0, 1500.0, 1760.0, 1750.0, 1750.0, 1980.0, 2010.0),
        /* safeMode = */ true
    ).createLUT()

    val hoodLUT: InterpLUT = InterpLUT(
        distances,
        listOf(0.0, 0.053, 0.38, 0.35, 0.31, 0.355, 0.332),
        true
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
        guess: Double = Math.toRadians(40.0),
        repetitions: Int = 3
    ): Double {
        val g = 9.81
        val height = 110.0 - 30.0
        if (repetitions <= 0) return guess
        val time = distance / (velocity * cos(guess))
        val f = velocity * sin(guess) * time - 0.5 * g * time * time - height
        val df = velocity * sin(guess) - g * time
        return findLaunchAngle(distance, velocity, guess - f / df, repetitions - 1)
    }

    private var oldVelocity = 0.0
    private var oldDistance = 0.0

    private fun update(distance: Double) {
        val position = encoder.currentPosition.toDouble()
        val velocity = encoder.velocity
        val desiredVelocity = controller.goal.velocity
        Log.v("ShooterImpl", "Velocity: $velocity, Desired: $desiredVelocity")
        setPower(controller.calculate(KineticState(position, velocity)))
        stateFlow.value = Shooter.State(velocity, abs(velocity - desiredVelocity) <= 80.0)
        if (abs(distance - oldDistance) > 0.1 || abs(velocity - oldVelocity) > 50.0) {
            hood = (Math.toDegrees(findLaunchAngle(distance, (desiredVelocity - 100.0) / 28.0)) - 33.0) /
                    (47.0 - 33.0)
            oldDistance = distance
            oldVelocity = velocity
        }
    }

    override fun shoot(distanceFlow: Flow<Double>): Job =
        opModeScope.launch {
            try {
                var dist = 0.0
                val updateJob = launch {
                    while (true) {
                        update(dist)
                        delay(50L)
                    }
                }
                distanceFlow.flowOn(Dispatchers.IO).collect { distance ->
                    dist = distance
                    val desiredVelocity = velocityLUT[distance]
//                    hood = (Math.toDegrees(findLaunchAngle(distance, (desiredVelocity - 100.0) / 28.0)) - 33.0) /
//                            (47.0 - 33.0)
                    controller.goal = KineticState(velocity = desiredVelocity)
                }
                updateJob.cancel()
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


package org.firstinspires.ftc.teamcode.pedropathing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.config.ValueProvider
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.zacsweers.metro.ContributesIntoMap
import dev.zacsweers.metro.MapKey
import dev.zacsweers.metro.Provider
import dev.zacsweers.metro.createGraphFactory
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.metro.OpModeGraph
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.abs
import kotlin.math.pow

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Config
@TeleOp(name = "Tuning", group = "Pedro Pathing")
class Tuning : SelectableOpMode("Select a Tuning OpMode", {
    for ((folderName, folderMap) in tuningOpModesMap) {
        folder(folderName) { folder ->
            for ((opModeName, opModeProvider) in folderMap) {
                folder.add(opModeName, opModeProvider)
            }
        }
    }
}) {
    val appGraph = createGraphFactory<OpModeGraph.Factory>().create(this)

    private val opModesMap = appGraph.tuningOpModesMap

    override fun init() {
        val finalMap = mutableMapOf<String, MutableMap<String, Provider<OpMode>>>()

        for ((key, value) in opModesMap) {
            finalMap.getOrPut(key.folder, ::mutableMapOf)[key.name] = value
        }

        tuningOpModesMap = finalMap
        this.telemetry = appGraph.telemetry
        super.init()
    }
}
private val changes = arrayListOf<String>()

private lateinit var tuningOpModesMap: Map<String, Map<String, Provider<OpMode>>>

fun drawOnlyCurrent(follower: Follower) {
    drawRobot(follower.pose)
    sendPacket()
}

fun stopRobot(follower: Follower) {
    follower.startTeleopDrive()
    follower.setTeleOpDrive(0.0, 0.0, 0.0)
}

@MapKey(unwrapValue = false)
annotation class TuningOpModeKey(val folder: String, val name: String)

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
 * You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Localization", name = "Localization Test")
class LocalizationTest(
    private val follower: Follower,
    @Suppress("PROPERTY_HIDES_JAVA_FIELD") private val telemetry: Telemetry
) : OpMode() {
    private var multiplier = 1.0

    override fun init() {}

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry.  */
    override fun init_loop() {
        telemetry.addLine(
            "This will print your robot's position to telemetry while "
                    + "allowing robot control through a basic mecanum drive on gamepad 1."
        )
        telemetry.update()
        drawOnlyCurrent(follower)
    }

    override fun start() {
        follower.startTeleopDrive()
        FtcDashboard.getInstance().run {
            addConfigVariable("Localization", "MULTIPLIER", object : ValueProvider<Double> {
                override fun get() = multiplier
                override fun set(value: Double) {
                    multiplier = value
                }
            })
            updateConfig()
        }
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    override fun loop() {
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y.toDouble() * multiplier,
            -gamepad1.left_stick_x.toDouble() * multiplier,
            -gamepad1.right_stick_x.toDouble(),
        )
        follower.update()

        telemetry.addData("Pose", follower.pose)
        telemetry.addData("Total Heading", follower.totalHeading)
        telemetry.update()

        drawDebug(follower)
    }
}

/**
 * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the forward ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Localization", name = "Forward Tuner")
class ForwardTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    override fun init() {
        follower.update()
        drawOnlyCurrent(follower)
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("Pull your robot forward $DISTANCE inches. Your forward ticks to inches will be shown on the telemetry.")
        telemetryA.update()
        drawOnlyCurrent(follower)
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        follower.update()

        telemetryA.addData("Distance Moved", follower.pose.x)
        telemetryA.addLine("The multiplier will display what your forward ticks to inches should be to scale your current distance to $DISTANCE inches.")
        telemetryA.addData(
            "Multiplier", (DISTANCE / (follower.pose.x / follower.poseTracker.localizer.forwardMultiplier))
        )
        telemetryA.update()

        drawDebug(follower)
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 48.0
    }
}

/**
 * This is the LateralTuner OpMode. This tracks the strafe movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the strafe ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 6/26/2025
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Localization", name = "Lateral Tuner")
class LateralTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    override fun init() {
        follower.update()
        drawOnlyCurrent(follower)
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("Pull your robot to the right $DISTANCE inches. Your strafe ticks to inches will be shown on the telemetry.")
        telemetryA.update()
        drawOnlyCurrent(follower)
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        follower.update()

        telemetryA.addData("Distance Moved", follower.pose.y)
        telemetryA.addLine("The multiplier will display what your strafe ticks to inches should be to scale your current distance to $DISTANCE inches.")
        telemetryA.addData(
            "Multiplier", (DISTANCE / (follower.pose.y / follower.poseTracker.localizer.lateralMultiplier))
        )
        telemetryA.update()

        drawDebug(follower)
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 48.0
    }
}

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Localization", name = "Turn Tuner")
class TurnTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    override fun init() {
        follower.update()
        drawOnlyCurrent(follower)
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("Turn your robot $ANGLE radians. Your turn ticks to inches will be shown on the telemetry.")
        telemetryA.update()

        drawOnlyCurrent(follower)
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        follower.update()

        telemetryA.addData("Total Angle", follower.totalHeading)
        telemetryA.addLine("The multiplier will display what your turn ticks to inches should be to scale your current angle to $ANGLE radians.")
        telemetryA.addData(
            "Multiplier", (ANGLE / (follower.totalHeading / follower.poseTracker.localizer.turningMultiplier))
        )
        telemetryA.update()

        drawDebug(follower)
    }

    companion object {
        @JvmField
        var ANGLE: Double = 2 * Math.PI
    }
}

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Automatic", name = "Forward Velocity Tuner")
class ForwardVelocityTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private val velocities = ArrayList<Double>()
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("The robot will run at 1 power until it reaches $DISTANCE inches forward.")
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.")
        telemetryA.addLine("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.")
        telemetryA.addLine("Press B on game pad 1 to stop.")
        telemetryA.addData("pose", follower.pose)
        telemetryA.update()

        follower.update()
        drawOnlyCurrent(follower)
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }
        follower.startTeleopDrive(true)
        follower.update()
        end = false
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot(follower)
            requestOpModeStop()
        }

        follower.update()
        drawDebug(follower)


        if (!end) {
            if (abs(follower.pose.x) > DISTANCE) {
                end = true
                stopRobot(follower)
            } else {
                follower.setTeleOpDrive(1.0, 0.0, 0.0, true)
                //double currentVelocity = Math.abs(follower.getVelocity().getXComponent());
                val currentVelocity = abs(follower.poseTracker.localizer.velocity.x)
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            stopRobot(follower)
            val average = velocities.average()
            telemetryA.addData("Forward Velocity", average)
            telemetryA.addLine("\n")
            telemetryA.addLine("Press A to set the Forward Velocity temporarily (while robot remains on).")

            for ((index, velocity) in velocities.withIndex()) {
                telemetryA.addData("Velocity $index", velocity)
            }

            telemetryA.update()
            telemetry.update()

            if (gamepad1.aWasPressed()) {
                follower.setXVelocity(average)
                changes += "XMovement: $average"
            }
        }
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 48.0
        @JvmField
        var RECORD_NUMBER: Double = 10.0
    }
}

/**
 * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot right at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Automatic", name = "Lateral Velocity Tuner")
class LateralVelocityTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private val velocities = arrayListOf<Double>()

    private var end = false

    override fun init() {}

    /**
     * This initializes the drive motors as well as the cache of velocities and the Panels
     * telemetryM.
     */
    override fun init_loop() {
        telemetryA.addLine("The robot will run at 1 power until it reaches $DISTANCE inches to the right.")
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.")
        telemetryA.addLine("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.")
        telemetryA.addLine("Press B on Gamepad 1 to stop.")
        telemetryA.update()

        follower.update()
        drawOnlyCurrent(follower)
    }

    /** This starts the OpMode by setting the drive motors to run right at full power.  */
    override fun start() {
        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }
        follower.startTeleopDrive(true)
        follower.update()
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run sideways enough, these last velocities recorded are
     * averaged and printed.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot(follower)
            requestOpModeStop()
        }

        follower.update()
        drawDebug(follower)

        if (!end) {
            if (abs(follower.pose.y) > DISTANCE) {
                end = true
                stopRobot(follower)
            } else {
                follower.setTeleOpDrive(0.0, 1.0, 0.0, true)
                val currentVelocity: Double = abs(follower.velocity.dot(Vector(1.0, Math.PI / 2)))
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            stopRobot(follower)
            val average = velocities.average()

            telemetryA.addData("Strafe Velocity", average)
            telemetryA.addLine("\n")
            telemetryA.addLine("Press A to set the Lateral Velocity temporarily (while robot remains on).")
            telemetryA.update()

            if (gamepad1.aWasPressed()) {
                follower.setYVelocity(average)
                changes += "YMovement: $average"
            }
        }
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 48.0
        @JvmField
        var RECORD_NUMBER: Double = 10.0
    }
}

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Automatic", name = "Forward Zero Power Acceleration Tuner")
class ForwardZeroPowerAccelerationTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private val accelerations = arrayListOf<Double>()
    private var previousVelocity = 0.0
    private var previousTimeNano: Long = 0

    private var stopping = false
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the Panels telemetryM.  */
    override fun init_loop() {
        telemetryA.addLine("The robot will run forward until it reaches $VELOCITY inches per second.")
        telemetryA.addLine("Then, it will cut power from the drivetrain and roll to a stop.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.addLine("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.")
        telemetryA.addLine("Press B on Gamepad 1 to stop.")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        follower.startTeleopDrive(false)
        follower.update()
        follower.setTeleOpDrive(1.0, 0.0, 0.0, true)
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot(follower)
            requestOpModeStop()
        }

        follower.update()
        drawDebug(follower)

        val heading = Vector(1.0, follower.pose.heading)
        if (!end) {
            if (!stopping) {
                if (follower.velocity.dot(heading) > VELOCITY) {
                    previousVelocity = follower.velocity.dot(heading)
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
                }
            } else {
                val currentVelocity: Double = follower.velocity.dot(heading)
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < follower.constraints.velocityConstraint) {
                    end = true
                }
            }
        } else {
            val average = accelerations.average()

            telemetryA.addData("Forward Zero Power Acceleration (Deceleration)", average)
            telemetryA.addLine("\n")
            telemetryA.addLine("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).")
            telemetryA.update()

            if (gamepad1.aWasPressed()) {
                follower.constants.forwardZeroPowerAcceleration = average
                changes += "Forward Zero Power Acceleration: $average"
            }
        }
    }

    companion object {
        @JvmField
        var VELOCITY: Double = 30.0
    }
}

/**
 * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * to the right until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Automatic", name = "Lateral Zero Power Acceleration Tuner")
class LateralZeroPowerAccelerationTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private val accelerations = ArrayList<Double>()
    private var previousVelocity = 0.0
    private var previousTimeNano: Long = 0
    private var stopping = false
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("The robot will run to the right until it reaches $VELOCITY inches per second.")
        telemetryA.addLine("Then, it will cut power from the drivetrain and roll to a stop.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.addLine("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.")
        telemetryA.addLine("Press B on game pad 1 to stop.")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        follower.startTeleopDrive(false)
        follower.update()
        follower.setTeleOpDrive(0.0, 1.0, 0.0, true)
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot(follower)
            requestOpModeStop()
        }

        follower.update()
        drawDebug(follower)

        val heading = Vector(1.0, follower.pose.heading - Math.PI / 2)
        if (!end) {
            if (!stopping) {
                if (abs(follower.velocity.dot(heading)) > VELOCITY) {
                    previousVelocity = abs(follower.velocity.dot(heading))
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
                }
            } else {
                val currentVelocity: Double = abs(follower.velocity.dot(heading))
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < follower.constraints.velocityConstraint) {
                    end = true
                }
            }
        } else {
            val average = accelerations.average()

            telemetryA.addData("Lateral Zero Power Acceleration (Deceleration)", average)
            telemetryA.addLine("\n")
            telemetryA.addLine("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).")
            telemetryA.update()

            if (gamepad1.aWasPressed()) {
                follower.getConstants().setLateralZeroPowerAcceleration(average)
                val message = "Lateral Zero Power Acceleration: $average"
                changes.add(message)
            }
        }
    }

    companion object {
        @JvmField
        var VELOCITY: Double = 30.0
    }
}

/**
 * This is the Translational PIDF Tuner OpMode. It will keep the robot in place.
 * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Manual", name = "Translational Tuner")
class TranslationalTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private var forward = true

    private val forwards = pathChain(follower) {
        pathConstantHeading(0.0) {
            +Pose()
            +Pose(DISTANCE, 0.0)
        }
    }

    private val backwards = pathChain(follower) {
        pathConstantHeading(0.0) {
            +Pose(DISTANCE, 0.0)
            +Pose()
        }
    }

    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        telemetryA.addLine("This will activate the translational PIDF(s)")
        telemetryA.addLine("The robot will try to stay in place while you push it laterally.")
        telemetryA.addLine("You can adjust the PIDF values to tune the robot's translational PIDF(s).")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    override fun start() {
        follower.deactivateAllPIDFs()
        follower.activateTranslational()
        follower.followPath(forwards)
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        follower.update()
        drawDebug(follower)

        if (!follower.isBusy) {
            if (forward) {
                forward = false
                follower.followPath(backwards)
            } else {
                forward = true
                follower.followPath(forwards)
            }
        }

        telemetryA.addLine("Push the robot laterally to test the Translational PIDF(s).")
        telemetryA.update()
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Heading PIDF Tuner OpMode. It will keep the robot in place.
 * The user should try to turn the robot to test the PIDF and adjust the PIDF values accordingly.
 * It will try to keep the robot at a constant heading while the user tries to turn it.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Manual", name = "Heading Tuner")
class HeadingTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private var forward = true

    private val forwards = pathChain(follower) {
        pathConstantHeading(0.0) {
            +Pose()
            +Pose(DISTANCE, 0.0)
        }
    }
    private val backwards = pathChain(follower) {
        pathConstantHeading(0.0) {
            +Pose(DISTANCE, 0.0)
            +Pose()
        }
    }

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    override fun init_loop() {
        telemetryA.addLine("This will activate the heading PIDF(s).")
        telemetryA.addLine("The robot will try to stay at a constant heading while you try to turn it.")
        telemetryA.addLine("You can adjust the PIDF values to tune the robot's heading PIDF(s).")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    override fun start() {
        follower.deactivateAllPIDFs()
        follower.activateHeading()
        follower.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        follower.update()
        drawDebug(follower)

        if (!follower.isBusy) {
            if (forward) {
                forward = false
                follower.followPath(backwards)
            } else {
                forward = true
                follower.followPath(forwards)
            }
        }

        telemetryA.addLine("Turn the robot manually to test the Heading PIDF(s).")
        telemetryA.update()
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Drive PIDF Tuner OpMode. It will run the robot in a straight line going forward and back.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Manual", name = "Drive Tuner")
class DriveTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private var forward = true

    private val forwards = pathChain(follower, decelerationType = PathChain.DecelerationType.GLOBAL) {
        pathConstantHeading(0.0) {
            +Pose()
            +Pose(DISTANCE, 0.0)
        }
    }
    private val backwards = pathChain(follower, decelerationType = PathChain.DecelerationType.GLOBAL) {
        pathConstantHeading(0.0) {
            +Pose(DISTANCE, 0.0)
            +Pose()
        }
    }

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    override fun init_loop() {
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE + "inches forward.")
        telemetryA.addLine("The robot will go forward and backward continuously along the path.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    override fun start() {
        follower.deactivateAllPIDFs()
        follower.activateDrive()
        follower.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        follower.update()
        drawDebug(follower)

        if (!follower.isBusy) {
            if (forward) {
                forward = false
                follower.followPath(backwards)
            } else {
                forward = true
                follower.followPath(forwards)
            }
        }

        telemetryA.addData("isForward", forward)
        telemetryA.update()
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Tests", name = "Line Test Tuner")
class Line(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private var forward = true

    private val forwards = pathChain(follower) {
        pathConstantHeading(0.0) {
            +Pose()
            +Pose(DISTANCE, 0.0)
        }
    }
    private val backwards = pathChain(follower) {
        pathConstantHeading(0.0) {
            +Pose(DISTANCE, 0.0)
            +Pose()
        }
    }

    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        telemetryA.addLine("This will activate all the PIDF(s)")
        telemetryA.addLine("The robot will go forward and backward continuously along the path while correcting.")
        telemetryA.addLine("You can adjust the PIDF values to tune the robot's drive PIDF(s).")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    override fun start() {
        follower.activateAllPIDFs()
        follower.followPath(forwards)
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        follower.update()
        drawDebug(follower)

        if (!follower.isBusy) {
            if (forward) {
                forward = false
                follower.followPath(backwards)
            } else {
                forward = true
                follower.followPath(forwards)
            }
        }

        telemetryA.addData("isForward", forward)
        telemetryA.update()
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Manual", name = "Centripetal Tuner")
class CentripetalTuner(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private var forward = true

    private val forwards = pathChain(follower) {
        path {
            +Pose()
            +Pose(abs(DISTANCE), 0.0)
            +Pose(abs(DISTANCE), DISTANCE)
        }
    }
    private val backwards = pathChain(follower) {
        path(interpolator = HeadingInterpolator.tangent.reverse()) {
            +Pose(abs(DISTANCE), DISTANCE)
            +Pose(abs(DISTANCE), 0.0)
            +Pose()
        }
    }

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths.
     * Additionally, this initializes the Panels telemetry.
     */
    override fun init_loop() {
        telemetryA.addLine("This will run the robot in a curve going $DISTANCE inches to the left and the same number of inches forward.")
        telemetryA.addLine("The robot will go continuously along the path.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    override fun start() {
        follower.activateAllPIDFs()
        follower.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        follower.update()
        drawDebug(follower)
        if (!follower.isBusy) {
            if (forward) {
                forward = false
                follower.followPath(backwards)
            } else {
                forward = true
                follower.followPath(forwards)
            }
        }

        telemetryA.addData("Driving away from the origin along the curve?", forward)
        telemetryA.update()
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 20.0
    }
}

/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Tests", name = "Triangle Test Tuner")
@OptIn(PathLinearExperimental::class)
class Triangle(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private val startPose = Pose(0.0, 0.0, Math.toRadians(0.0))
    private val interPose = Pose(24.0, -24.0, Math.toRadians(90.0))
    private val endPose = Pose(24.0, 24.0, Math.toRadians(45.0))

    private val triangle = pathChain(follower) {
        pathLinearHeading {
            +startPose
            +interPose
        }
        pathLinearHeading {
            +interPose
            +endPose
        }
        pathLinearHeading {
            +endPose
            +startPose
        }
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        follower.update()
        drawDebug(follower)

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true)
        }
    }

    override fun init() {}

    override fun init_loop() {
        telemetryA.addLine("This will run in a roughly triangular shape, starting on the bottom-middle point.")
        telemetryA.addLine("So, make sure you have enough space to the left, front, and right to run the OpMode.")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    /** Creates the PathChain for the "triangle". */
    override fun start() {
        follower.setStartingPose(startPose)

        follower.followPath(triangle)
    }
}

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bézier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Tests", name = "Circle Test Tuner")
class Circle(private val follower: Follower, private val telemetryA: Telemetry) : OpMode() {
    private val circle = pathChain(follower) {
        pathFacingPoint(0.0, RADIUS) {
            +Pose()
            +Pose(RADIUS, 0.0)
            +Pose(RADIUS, RADIUS)
        }
        pathFacingPoint(0.0, RADIUS) {
            +Pose(RADIUS, RADIUS)
            +Pose(RADIUS, 2 * RADIUS)
            +Pose(0.0, 2 * RADIUS)
        }
        pathFacingPoint(0.0, RADIUS) {
            +Pose(0.0, 2 * RADIUS)
            +Pose(-RADIUS, 2 * RADIUS)
            +Pose(-RADIUS, RADIUS)
        }
        pathFacingPoint(0.0, RADIUS) {
            +Pose(-RADIUS, RADIUS)
            +Pose(-RADIUS, 0.0)
            +Pose()
        }
    }

    override fun start() {
        follower.followPath(circle)
    }

    override fun init_loop() {
        telemetryA.addLine("This will run in a roughly circular shape of radius $RADIUS, starting on the right-most edge. ")
        telemetryA.addLine("So, make sure you have enough space to the left, front, and back to run the OpMode.")
        telemetryA.addLine("It will also continuously face the center of the circle to test your heading and centripetal correction.")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent(follower)
    }

    override fun init() {}

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    override fun loop() {
        follower.update()
        drawDebug(follower)

        if (follower.atParametricEnd()) {
            follower.followPath(circle)
        }
    }

    companion object {
        var RADIUS: Double = 10.0
    }
}

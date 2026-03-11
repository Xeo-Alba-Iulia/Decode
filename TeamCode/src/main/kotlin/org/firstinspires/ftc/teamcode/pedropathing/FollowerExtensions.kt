package org.firstinspires.ftc.teamcode.pedropathing

import android.util.Log
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.take
import kotlinx.coroutines.selects.onTimeout
import kotlinx.coroutines.selects.select
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.auto.FarAuto.Companion.TAG
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

suspend fun Follower.followSuspend(pathChain: PathChain, holdEnd: Boolean = constants.automaticHoldEnd) {
    followPath(pathChain, holdEnd)
    while (isBusy) {
        update()
        yield()
    }
}

fun Follower.followSuspendFlow(pathChain: PathChain, holdEnd: Boolean = constants.automaticHoldEnd) = flow {
    followPath(pathChain, holdEnd)
    while (isBusy) {
        update()
        emit(pose!!)
        yield()
    }
    emit(pose!!)
}

suspend fun Follower.followSuspend(
    pathChain: PathChain,
    maxPower: Double,
    holdEnd: Boolean = constants.automaticHoldEnd
) {
    followPath(pathChain, maxPower, holdEnd)
    while (isBusy) {
        update()
        yield()
    }
}

fun Follower.followSuspendFlow(
    pathChain: PathChain,
    maxPower: Double,
    holdEnd: Boolean = constants.automaticHoldEnd
) = flow {
    followPath(pathChain, maxPower, holdEnd)
    while (isBusy) {
        update()
        emit(pose!!)
        yield()
    }
    emit(pose!!)
}

suspend fun Follower.holdSuspend(pose: Pose, timeout: Duration) {
    holdPoint(pose)
    withTimeout(timeout) {
        while (true) {
            update()
            yield()
        }
    }
}

/**
 * Follow the given path while intaking, and stop when either the path is finished or the timeout is reached.
 *
 * This is the most general version of followAndIntake, which takes a suspend function as a parameter
 * to allow for maximum flexibility in how the path is followed.
 *
 * The other versions of followAndIntake are just convenience wrappers around this
 * one that call it with different parameters.
 */
@OptIn(ExperimentalCoroutinesApi::class)
suspend inline fun Follower.followAndIntake(
    intake: Intake,
    sorter: Sorter,
    colorList: List<ArtefactType> = List(3) { PURPLE },
    timeout: Duration = 5.seconds,
    crossinline followFunction: suspend Follower.() -> Unit,
): Unit =
    coroutineScope {
        intake.isRunning = true
        val intakeJob = launch {
            delay(500L)
            Log.d(TAG, "Intake started, size is: ${sorter.size}")
            val listIter = colorList.iterator()
            intake.distanceFlow
                .filter { it }
                .take(3 - sorter.size)
                .collect {
                    sorter.intake(listIter.next())
                    RobotLog.dd(TAG, "Intake $it, sorter now has ${sorter.size} artefacts")
                    delay(200L)
                }
        }
        val followerJob = launch { followFunction() }
        select {
            intakeJob.onJoin { Log.d(TAG, "Stopped follow because intake finished") }
            followerJob.onJoin {
                Log.e(TAG, "Path finished, only picked up ${sorter.size} artefacts")
            }
            onTimeout(timeout) { Log.e(TAG, "Path didn't finish, picked up ${sorter.size} artefacts") }
        }
        followerJob.cancel()
        intakeJob.cancel()
        intake.isServoRunning = true
    }

/**
 * Follow the given path while intaking, and stop when either the path is finished or the timeout is reached.
 */
suspend inline fun Follower.followAndIntake(
    intake: Intake,
    sorter: Sorter,
    path: PathChain,
    colorList: List<ArtefactType> = List(3) { PURPLE },
    timeout: Duration = 5.seconds,
) = followAndIntake(intake, sorter, colorList, timeout) { followSuspend(path) }

/**
 * Deprecated version of followAndIntake that doesn't take a path.
 *
 * This is used to cause a compile error if someone tries to call followAndIntake without a path,
 * since that would be a bug.
 *
 * @throws UnsupportedOperationException
 */
@Suppress("UNUSED_PARAMETER", "UnusedReceiverParameter")
@Deprecated("No path provided", level = DeprecationLevel.ERROR)
fun Follower.followAndIntake(
    intake: Intake,
    sorter: Sorter,
    colorList: List<ArtefactType> = emptyList(),
    timeout: Duration = 5.seconds
): Unit = throw UnsupportedOperationException("No path provided to follow")

/**
 * Follow the given paths while intaking, and stop when either the paths are finished or the timeout is reached.
 *
 * This is a vararg version of followAndIntake that allows you to provide multiple paths to follow in sequence.
 * To set max powers individually, they must be set in the paths callbacks themselves.
 */
suspend inline fun Follower.followAndIntake(
    intake: Intake,
    sorter: Sorter,
    vararg paths: PathChain,
    colorList: List<ArtefactType> = List(3) { PURPLE },
    timeout: Duration = 5.seconds,
) = followAndIntake(intake, sorter, colorList, timeout) {
    assert(paths.isNotEmpty())
    paths.forEach { followSuspend(it) }
}
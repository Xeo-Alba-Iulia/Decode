package org.firstinspires.ftc.teamcode.pedropathing

import android.util.Log
import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.take
import kotlinx.coroutines.selects.onTimeout
import kotlinx.coroutines.selects.select
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.auto.FarAuto.Companion.TAG
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.contracts.ExperimentalContracts
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

private val followingMutex = Mutex()

suspend fun Follower.followSuspend(pathChain: PathChain, holdEnd: Boolean = constants.automaticHoldEnd) =
    followingMutex.withLock {
        followPath(pathChain, holdEnd)
        while (isBusy) {
            update()
            yield()
        }
    }

suspend fun Follower.followSuspend(
    pathChain: PathChain,
    maxPower: Double,
    holdEnd: Boolean = constants.automaticHoldEnd
) = followingMutex.withLock {
    followPath(pathChain, maxPower, holdEnd)
    while (isBusy) {
        update()
        yield()
    }
}

@OptIn(ExperimentalCoroutinesApi::class, ExperimentalContracts::class)
suspend inline fun Follower.followAndIntake(
    intake: Intake,
    sorter: Sorter,
    timeout: Duration = 5.seconds,
    crossinline followFunction: suspend Follower.() -> Unit,
): Unit =
    coroutineScope {
        intake.isRunning = true
        val intakeJob = launch {
            intake.artefactFlow.take(3 - sorter.size).collect {
                sorter.intake(it)
                RobotLog.dd(TAG, "Intake $it, sorter now has ${sorter.size} artefacts")
                delay(200L)
            }
        }
        val followerJob = launch { followFunction() }
        var pathFinished = false
        select {
            intakeJob.onJoin { Log.d(TAG, "Stopped follow because intake finished") }
            followerJob.onJoin {
                Log.e(TAG, "Path finished, only picked up ${sorter.size} artefacts")
                pathFinished = true
            }
            onTimeout(timeout) { Log.e(TAG, "Path didn't finish, picked up ${sorter.size} artefacts") }
        }
        followerJob.cancel()
        if (pathFinished)
            delay(500.milliseconds)
        intakeJob.cancel()
        intake.isServoRunning = true
    }

suspend inline fun Follower.followAndIntake(
    intake: Intake,
    sorter: Sorter,
    path: PathChain,
    timeout: Duration = 5.seconds,
) = followAndIntake(intake, sorter, timeout) { followSuspend(path) }

suspend inline fun Follower.followAndIntake(
    intake: Intake,
    sorter: Sorter,
    vararg paths: PathChain,
    timeout: Duration = 5.seconds,
) = followAndIntake(intake, sorter, timeout) { require(paths.isNotEmpty()); paths.forEach { followSuspend(it) } }
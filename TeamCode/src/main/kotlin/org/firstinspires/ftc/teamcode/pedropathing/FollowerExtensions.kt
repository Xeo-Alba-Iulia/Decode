package org.firstinspires.ftc.teamcode.pedropathing

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.yield

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
package org.firstinspires.ftc.teamcode.pedropathing

import com.pedropathing.follower.Follower
import com.pedropathing.paths.PathChain
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.yield

private val followingMutex = Mutex()

suspend fun Follower.followSuspend(pathChain: PathChain) = followingMutex.withLock {
    followPath(pathChain)
    while (isBusy) yield()
}

suspend fun Follower.followSuspend(pathChain: PathChain, holdEnd: Boolean) = followingMutex.withLock {
    followPath(pathChain, holdEnd)
    while (isBusy) yield()
}

suspend fun Follower.followSuspend(pathChain: PathChain, maxPower: Double, holdEnd: Boolean) = followingMutex.withLock {
    followPath(pathChain, maxPower, holdEnd)
    while (isBusy) yield()
}
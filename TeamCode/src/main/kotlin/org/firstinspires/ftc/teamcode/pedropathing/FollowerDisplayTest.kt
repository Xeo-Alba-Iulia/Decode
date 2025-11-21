package org.firstinspires.ftc.teamcode.pedropathing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.RobotLog
import dev.zacsweers.metro.ContributesIntoMap
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Custom", name = FollowerDisplayTest.TAG)
class FollowerDisplayTest(follower: Follower) : OpMode() {
    companion object {
        lateinit var follower: Follower
        const val TAG = "FollowerDisplayTest"
    }

    init {
        Companion.follower = follower
    }

    override fun init() {
        RobotLog.dd(TAG, "Started initialization")
        FtcDashboard.getInstance().withConfigRoot {
            it.putVariable(TAG, ReflectionConfig.createVariableFromClass(this.javaClass))
        }
        RobotLog.dd(TAG, "Finished initialization")
    }

    override fun loop() {}
}
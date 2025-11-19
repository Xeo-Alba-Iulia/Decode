package org.firstinspires.ftc.teamcode.pedropathing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.VariableProvider
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.RobotLog
import dev.zacsweers.metro.ContributesIntoMap
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@ContributesIntoMap(OpModeScope::class)
@TuningOpModeKey(folder = "Custom", name = FollowerDisplayedTest.TAG)
class FollowerDisplayedTest(private val follower: Follower) : OpMode() {
    companion object {
        const val TAG = "FollowerDisplayTest"
    }

    override fun init() {
        RobotLog.dd(TAG, "Started")
        FtcDashboard.getInstance().addConfigVariable("FollowerDisplayTest", "follower", VariableProvider(follower))
        RobotLog.dd(TAG, "Added")
        FtcDashboard.getInstance().updateConfig()
    }

    override fun loop() {}
}
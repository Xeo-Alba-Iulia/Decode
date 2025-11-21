package org.firstinspires.ftc.teamcode.pedropathing

import com.bylazar.configurables.PanelsConfigurables
import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.zacsweers.metro.createGraphFactory
import org.firstinspires.ftc.teamcode.metro.OpModeGraph

//@ContributesIntoMap(OpModeScope::class)
//@TuningOpModeKey(folder = "Custom", name = FollowerDisplayedTest.TAG)
@TeleOp
@Configurable
class FollowerDisplayedTest : OpMode() {
    companion object {
        var follower: Follower? = null
        const val TAG = "FollowerDisplayTest"
    }

    lateinit var opModeGraph: OpModeGraph

    override fun init() {
        opModeGraph = createGraphFactory<OpModeGraph.Factory>().create(this)
        follower = opModeGraph.follower
        PanelsConfigurables.refreshClass(this)
    }

    override fun loop() {}
}
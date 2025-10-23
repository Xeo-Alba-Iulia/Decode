package org.firstinspires.ftc.teamcode.pedropathing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.PoseHistory

const val ROBOT_RADIUS: Double = 9.0

private var packet: TelemetryPacket? = null

/**
 * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
 * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
 *
 * @param follower
 */
fun drawDebug(follower: Follower) {
    if (follower.currentPath != null) {
        drawPath(follower.currentPath, "#3F51B5")
        val closestPoint = follower.getPointFromPath(follower.currentPath.closestPointTValue)
        drawRobot(
            Pose(
                closestPoint.x,
                closestPoint.y,
                follower.currentPath.getHeadingGoal(follower.currentPath.closestPointTValue)
            ), "#3F51B5"
        )
    }
    drawPoseHistory(follower.poseHistory, "#4CAF50")
    drawRobot(follower.pose, "#4CAF50")

    sendPacket()
}

/**
 * This adds instructions to the current packet to draw a robot at a specified Pose with a specified
 * color. If no packet exists, then a new one is created.
 *
 * @param pose the Pose to draw the robot at
 * @param color the color to draw the robot with
 */
@JvmOverloads
fun drawRobot(pose: Pose, color: String = "#4CAF50") {
    if (packet == null) packet = TelemetryPacket()

    packet!!.fieldOverlay().setStroke(color)
    drawRobotOnCanvas(packet!!.fieldOverlay(), pose.copy())
}

/**
 * This adds instructions to the current packet to draw a Path with a specified color. If no
 * packet exists, then a new one is created.
 *
 * @param path the Path to draw
 * @param color the color to draw the Path with
 */
fun drawPath(path: Path, color: String) {
    if (packet == null) packet = TelemetryPacket()

    packet!!.fieldOverlay().setStroke(color)
    drawPath(packet!!.fieldOverlay(), path.panelsDrawingPoints)
}

/**
 * This adds instructions to the current packet to draw all the Paths in a PathChain with a
 * specified color. If no packet exists, then a new one is created.
 *
 * @param pathChain the PathChain to draw
 * @param color the color to draw the PathChain with
 */
fun drawPath(pathChain: PathChain, color: String) {
    for (i in 0..<pathChain.size()) {
        drawPath(pathChain.getPath(i), color)
    }
}

/**
 * This adds instructions to the current packet to draw the pose history of the robot. If no
 * packet exists, then a new one is created.
 *
 * @param poseHistory the [PoseHistory] to get the pose history from
 * @param color the color to draw the pose history with
 */
fun drawPoseHistory(poseHistory: PoseHistory, color: String) {
    if (packet == null) packet = TelemetryPacket()

    packet!!.fieldOverlay().setStroke(color)
    packet!!.fieldOverlay().strokePolyline(poseHistory.xPositionsArray, poseHistory.yPositionsArray)
}

/**
 * This tries to send the current packet to FTC Dashboard.
 *
 * @return returns if the operation was successful.
 */
fun sendPacket(): Boolean {
    if (packet != null) {
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
        packet = null
        return true
    }
    return false
}

/**
 * This draws a robot on the Dashboard at a specified Pose. This is more useful for drawing the
 * actual robot, since the Pose contains the direction the robot is facing as well as its position.
 *
 * @param c the Canvas on the Dashboard on which this will draw at
 * @param t the Pose to draw at
 */
fun drawRobotOnCanvas(c: Canvas, t: Pose) {
    c.strokeCircle(t.x, t.y, ROBOT_RADIUS)
    val v: Vector = t.headingAsUnitVector * ROBOT_RADIUS
    val x1 = t.x + v.xComponent / 2
    val y1 = t.y + v.yComponent / 2
    val x2 = t.x + v.xComponent
    val y2 = t.y + v.yComponent
    c.strokeLine(x1, y1, x2, y2)
}

/**
 * This draws a Path on the Dashboard from a specified Array of Points.
 *
 * @param c the Canvas on the Dashboard on which this will draw
 * @param points the Points to draw
 */
fun drawPath(c: Canvas, points: Array<DoubleArray>) {
    c.strokePolyline(points[0], points[1])
}

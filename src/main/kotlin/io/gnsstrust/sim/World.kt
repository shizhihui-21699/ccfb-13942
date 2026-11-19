package io.gnsstrust.sim

import io.gnsstrust.model.*
import kotlin.math.*

/**
 * 场景生成: 把一个场景 + 一条轨迹 + 一个攻击模型拼成完整的传感器流。
 *
 * 这是唯一能看到真值的地方。检测器只拿到 [ScenarioMeta.frames]。
 */
object World {

    data class TrackBundle(
        val track: TruthTrack,
        val towers: List<Tower>,
        val aps: List<Ap>,
        val anchors: List<Anchor>,
        val roads: RoadNetwork,
    )

    fun build(spec: ScenarioSpec): ScenarioMeta {
        val bundle = trackFor(spec)
        val track = bundle.track

        val sim = SensorSim(spec.seed, spec.platform)
        sim.towers = bundle.towers
        sim.aps = bundle.aps
        sim.anchors = bundle.anchors

        val frames = ArrayList<SensorFrame>(track.n)
        val truth = ArrayList<TruthState>(track.n)
        val atk = ArrayList<AttackOffset>(track.n)
        val outages = ArrayList<ClosedFloatingPointRange<Double>>()
        val jams = ArrayList<ClosedFloatingPointRange<Double>>()

        var openStart = Double.NaN
        var jamStart = Double.NaN
        var lastValid = true
        var lastJam = false

        for (i in 0 until track.n) {
            val tr = track.state(i)
            val a = spec.attack.at(i, tr.t, tr, track)
            var f = sim.frame(i, tr, a, track.dt)
            // 人为安排的失锁: 隧道/地库之外也可能因为遮挡短暂丢星
            if (track.forcedOutage[i] && f.gnss.valid) {
                f = f.copy(gnss = GnssFix.invalid(tr.t))
            }
            frames.add(f)
            truth.add(tr)
            atk.add(AttackOffset(tr.t, a.dE, a.dN, a.dAlt, a.active, a.kind))

            val valid = f.gnss.valid
            if (lastValid && !valid) openStart = tr.t
            if (!lastValid && valid) {
                // 只记录"真正的失锁", 单帧丢星不算
                if (tr.t - openStart >= 0.5) outages.add(openStart..tr.t)
                openStart = Double.NaN
            }
            lastValid = valid
            if (lastJam && !a.jam) { jams.add(jamStart..tr.t); jamStart = Double.NaN }
            if (!lastJam && a.jam) jamStart = tr.t
            lastJam = a.jam
        }
        if (!lastValid && !openStart.isNaN() && track.n * track.dt - openStart >= 0.5)
            outages.add(openStart..track.n * track.dt)
        if (lastJam && !jamStart.isNaN()) jams.add(jamStart..track.n * track.dt)

        // 无人机: 把计划航线作为"任务走廊"交给检测器 (地图约束).
        // 真实系统里这就是航路规划下发的航线 + 容差带。
        val corridorPt = if (spec.platform == Platform.DRONE) bundle.track.path.sampled(20.0) else null

        return ScenarioMeta(
            name = spec.name, platform = spec.platform, attackKind = spec.attack.kind,
            seed = spec.seed, durationS = track.n * track.dt,
            truth = truth, attack = atk, frames = frames,
            outageWindows = outages, jammedWindows = jams,
            roads = bundle.roads,
            corridor = corridorPt,
            corridorHalfWidthM = if (spec.platform == Platform.DRONE) 60.0 else Double.NaN,
        )
    }

    private fun trackFor(spec: ScenarioSpec): TrackBundle = when (spec.platform) {
        Platform.DRONE -> {
            val sc = DroneScene(spec.seed)
            val v = SpeedProfile.build(
                sc.route, alatMax = 2.0, accelMax = 1.2, decelMax = 1.6, dt = 0.1,
                vmaxAt = sc::vmaxAt, stopAt = sc.stops(),
            )
            val tr = TruthTrack.build(
                platform = Platform.DRONE, path = sc.route, vProfile = v,
                durationS = spec.durationS, seed = spec.seed,
                envAt = sc.envMap::at, altProfile = sc::altAt,
                dwellStops = listOf(sc.route.total - 8.0 to 10.0),
                dt = 0.1,
            )
            TrackBundle(tr, sc.towers, sc.aps, AnchorNetwork.forDrone(spec.seed), sc.roads)
        }

        Platform.TRUCK -> {
            val sc = TruckScene(spec.seed)
            val stops = if (spec.stationaryWholeTime) emptyList() else listOf(
                sc.route.total * 0.30 to 22.0,
                sc.route.total * 0.62 to 15.0,
                sc.route.total - 6.0 to 30.0,
            )
            val v = SpeedProfile.build(
                sc.route, alatMax = 2.6, accelMax = 1.5, decelMax = 2.2, dt = 0.1,
                vmaxAt = sc::vmaxAt,
                stopAt = if (spec.stationaryWholeTime) emptyList() else sc.stops(),
            )
            val tr = TruthTrack.build(
                platform = Platform.TRUCK, path = sc.route, vProfile = v,
                durationS = spec.durationS, seed = spec.seed,
                envAt = sc.envMap::at, dwellStops = stops,
                stationaryFrom = if (spec.stationaryWholeTime) 0.0 else Double.MAX_VALUE,
                forceOutage = spec.forceOutage,
                dt = 0.1,
            )
            TrackBundle(tr, sc.towers, sc.aps, AnchorNetwork.forTruck(spec.seed), sc.roads)
        }
    }
}

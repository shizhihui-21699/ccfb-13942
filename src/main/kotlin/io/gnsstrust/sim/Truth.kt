package io.gnsstrust.sim

import io.gnsstrust.core.SimRandom
import io.gnsstrust.model.EnvClass
import io.gnsstrust.model.Platform
import io.gnsstrust.model.TruthState
import kotlin.math.*

/**
 * 真值轨迹生成。
 *
 * 所有传感器都在这个真值上叠加误差模型 —— 检测器永远看不到它。
 * 评估时用它算"GNSS 报了 (e,n) 而真实在 (e0,n0), 被拉偏了多少米"。
 *
 * 车体坐标约定 (全模块一致): x 前, y 左, z 上。
 * 航向 ψ 用数学角: 0 = 东, 逆时针为正。
 */
class TruthTrack(
    val platform: Platform,
    val path: Path,
    val dt: Double,
    val n: Int,
    val e: DoubleArray, val nArr: DoubleArray,
    val vE: DoubleArray, val vN: DoubleArray,
    val psi: DoubleArray,
    val alt: DoubleArray,
    val yawRate: DoubleArray,
    val aLong: DoubleArray,
    val aLat: DoubleArray,
    val aVert: DoubleArray,
    val vFwd: DoubleArray,
    val env: Array<EnvClass>,
    val dist: DoubleArray,
    val sAlong: DoubleArray,
    val stationary: BooleanArray,
    /** 人为安排的 GNSS 失锁 (仿真用, 验证"失锁期间不判") */
    val forcedOutage: BooleanArray = BooleanArray(n),
) {
    fun state(i: Int): TruthState = TruthState(
        t = i * dt, e = e[i], north = nArr[i], vE = vE[i], vN = vN[i],
        psi = psi[i], altM = alt[i], env = env[i], trueSpeedMps = vFwd[i],
        yawRate = yawRate[i], aLong = aLong[i], aLat = aLat[i], aVert = aVert[i],
        distanceTravelled = dist[i],
    )

    fun indexAt(t: Double): Int = (t / dt).roundToInt().coerceIn(0, n - 1)

    companion object {
        /**
         * 沿路径积分速度剖面得到真值。
         *
         * @param dwellAtStopS 在哪些路径里程处停车等待 (红灯/装卸), 单位秒
         * @param altProfile 高度剖面, 仅无人机使用; null 表示地面
         */
        fun build(
            platform: Platform,
            path: Path,
            vProfile: DoubleArray,
            durationS: Double,
            seed: Long,
            envAt: (Double, Double) -> EnvClass,
            altProfile: ((Double) -> Double)? = null,
            dwellStops: List<Pair<Double, Double>> = emptyList(),
            stationaryFrom: Double = Double.MAX_VALUE,
            forceOutage: List<ClosedFloatingPointRange<Double>> = emptyList(),
            dt: Double = 0.1,
        ): TruthTrack {
            val n = (durationS / dt).toInt() + 1
            val R = SimRandom(seed * 13 + 5)
            val dsGrid = path.total / (vProfile.size - 1)
            fun vAtS(s: Double): Double {
                val x = (s / dsGrid).coerceIn(0.0, (vProfile.size - 1).toDouble())
                val i = x.toInt().coerceAtMost(vProfile.size - 2)
                val f = x - i
                return vProfile[i] * (1 - f) + vProfile[i + 1] * f
            }

            val e = DoubleArray(n); val nn = DoubleArray(n)
            val vE = DoubleArray(n); val vN = DoubleArray(n)
            val psiA = DoubleArray(n); val altA = DoubleArray(n)
            val yawRate = DoubleArray(n); val aLongA = DoubleArray(n)
            val aLatA = DoubleArray(n); val aVertA = DoubleArray(n)
            val vFwd = DoubleArray(n); val envA = Array(n) { EnvClass.OPEN_SKY }
            val distA = DoubleArray(n); val sA = DoubleArray(n)
            val stationary = BooleanArray(n)
            val forced = BooleanArray(n)
            for (i in 0 until n) { val tt = i * dt; forced[i] = forceOutage.any { tt in it } }

            var s = 0.0
            var travelled = 0.0
            var vPrev = 0.0
            var altPrev = altProfile?.invoke(0.0) ?: 0.0

            // 速度扰动: 一个慢变 OU 过程, 让"匀速"不像仿真那样完美
            var vOsc = 0.0

            var holdUntil = -1.0
            var holdS = 0.0

            for (i in 0 until n) {
                val t = i * dt
                // ---- 停车/驻车逻辑
                var inHold = false
                if (t < holdUntil) {
                    inHold = true
                    s = holdS
                } else {
                    for ((stopS, dwell) in dwellStops) {
                        if (s < stopS && s + 1e-9 >= stopS - 0.35 && dwell > 0) {
                            holdUntil = t + dwell
                            holdS = stopS
                            s = stopS
                            inHold = true
                            break
                        }
                    }
                }
                val parked = t >= stationaryFrom

                // 注意: 出发时 vAtS(0)=0, 若直接照抄速度剖面就会永久卡在原点。
                // 这里给一个起步加速度, 由路径前方 6 m 处的剖面值回升来驱动。
                val vTarget = if (inHold || parked) 0.0 else max(vAtS(s), vAtS(s + 6.0) * 0.7)
                vOsc += (-vOsc / 8.0) * dt + 0.06 * sqrt(dt) * R.gauss()
                // 路径已走完 => 停住 (降落/到站), 速度必须为 0
                val atPathEnd = s >= path.total - 1e-6
                val v = if (inHold || parked || atPathEnd) 0.0 else max(0.0, vTarget * (1.0 + 0.03 * vOsc))

                val p = path.at(s)
                val kappa = p[4]
                val psiVal = atan2(p[3], p[2])

                val aLong = if (inHold || parked || atPathEnd) 0.0 else (v - vPrev) / dt
                val aLat = if (inHold || parked || atPathEnd) 0.0 else v * v * kappa
                val yr = if (inHold || parked || atPathEnd) 0.0 else v * kappa

                val altVal = altProfile?.invoke(s) ?: 0.0
                val aVert = if (i == 0) 0.0 else (altVal - altPrev) / dt

                e[i] = p[0]; nn[i] = p[1]
                vE[i] = v * cos(psiVal); vN[i] = v * sin(psiVal)
                psiA[i] = psiVal
                altA[i] = altVal
                yawRate[i] = yr
                aLongA[i] = aLong
                aLatA[i] = aLat
                aVertA[i] = aVert
                vFwd[i] = v
                envA[i] = envAt(p[0], p[1])
                distA[i] = travelled
                sA[i] = s
                stationary[i] = (v < 0.15)

                if (i > 0) travelled += hypot(e[i] - e[i - 1], nn[i] - nn[i - 1])
                vPrev = v
                altPrev = altVal

                // 路径走完后停在终点 (无人机降落/车辆到站)
                if (!inHold && !parked && !atPathEnd) s = min(path.total, s + v * dt)
            }

            return TruthTrack(
                platform, path, dt, n, e, nn, vE, vN, psiA, altA,
                yawRate, aLongA, aLatA, aVertA, vFwd, envA, distA, sA, stationary, forced
            )
        }
    }
}

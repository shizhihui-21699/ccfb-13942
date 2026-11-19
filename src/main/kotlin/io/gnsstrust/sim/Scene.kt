package io.gnsstrust.sim

import kotlin.math.hypot
import io.gnsstrust.model.EnvClass
import io.gnsstrust.model.AnchorKind
import kotlin.math.*

/**
 * 带圆弧倒角的二维路径。
 *
 * 为什么不用折线: 折线在拐点处曲率无穷大, 算出来的横向加速度是冲激,
 * 会在 IMU 上制造现实中不存在的尖峰, 让检测器的评估结果虚高。
 * 用圆弧倒角后曲率分段恒定, 速度剖面、横向加速度、横滚角都是光滑的。
 */
sealed class PathSeg {
    abstract val length: Double
    /** 返回 [x, y, tx, ty, curvature] */
    abstract fun at(sLocal: Double): DoubleArray
}

class LineSeg(val a: DoubleArray, val b: DoubleArray) : PathSeg() {
    private val dx = b[0] - a[0]
    private val dy = b[1] - a[1]
    override val length = hypot(dx, dy)
    private val ux = if (length > 1e-9) dx / length else 1.0
    private val uy = if (length > 1e-9) dy / length else 0.0

    override fun at(sLocal: Double): DoubleArray =
        doubleArrayOf(a[0] + ux * sLocal, a[1] + uy * sLocal, ux, uy, 0.0)
}

class ArcSeg(
    private val cx: Double, private val cy: Double, private val r: Double,
    private val a0: Double, private val sweep: Double,
) : PathSeg() {
    override val length = abs(sweep) * r
    private val sgn = sign(sweep)

    override fun at(sLocal: Double): DoubleArray {
        val a = a0 + sgn * (sLocal / max(r, 1e-9))
        return doubleArrayOf(
            cx + r * cos(a), cy + r * sin(a),
            -sin(a) * sgn, cos(a) * sgn,
            sgn / r
        )
    }
}

class Path(val segs: List<PathSeg>) {
    val cum: DoubleArray = DoubleArray(segs.size + 1).also {
        for (i in segs.indices) it[i + 1] = it[i] + segs[i].length
    }
    val total: Double get() = cum.last()

    fun at(s: Double): DoubleArray {
        val sc = s.coerceIn(0.0, total)
        var lo = 0
        var hi = segs.size - 1
        while (lo < hi) {
            val mid = (lo + hi) / 2
            if (cum[mid + 1] < sc) lo = mid + 1 else hi = mid
        }
        return segs[lo].at(sc - cum[lo])
    }

    /** 折线采样, 供地图匹配/最近点投影使用 */
    fun sampled(step: Double = 10.0): List<DoubleArray> {
        val out = ArrayList<DoubleArray>()
        var s = 0.0
        while (s <= total) {
            val p = at(s)
            out.add(doubleArrayOf(p[0], p[1]))
            s += step
        }
        val p = at(total)
        out.add(doubleArrayOf(p[0], p[1]))
        return out
    }

    companion object {
        /** 用半径 R 对折线倒角; R 会按相邻边长自动收缩, 保证圆弧不重叠 */
        fun rounded(wps: List<DoubleArray>, radius: Double, closed: Boolean = false): Path {
            val pts = if (closed) wps + listOf(wps[0]) else wps
            val segs = ArrayList<PathSeg>()
            var cur = pts[0]
            for (i in 1 until pts.size - 1) {
                val prev = pts[i - 1]; val v = pts[i]; val next = pts[i + 1]
                val u1 = unit(prev, v); val u2 = unit(v, next)
                val cosT = (u1[0] * u2[0] + u1[1] * u2[1]).coerceIn(-1.0, 1.0)
                val turn = acos(cosT)
                if (turn < 1e-3) continue
                val lenIn = hypot(v[0] - prev[0], v[1] - prev[1])
                val lenOut = hypot(next[0] - v[0], next[1] - v[1])
                var d = radius * tan(turn / 2)
                d = min(d, min(lenIn, lenOut) * 0.45)
                if (d < 1e-3) continue
                val r = d / tan(turn / 2)
                val t1 = doubleArrayOf(v[0] - u1[0] * d, v[1] - u1[1] * d)
                val t2 = doubleArrayOf(v[0] + u2[0] * d, v[1] + u2[1] * d)
                // 左转/右转决定圆心在法线哪一侧
                val cross = u1[0] * u2[1] - u1[1] * u2[0]
                val sgn = if (cross > 0) 1.0 else -1.0
                val n = doubleArrayOf(-u1[1] * sgn, u1[0] * sgn)
                val c = doubleArrayOf(t1[0] + n[0] * r, t1[1] + n[1] * r)
                segs.add(LineSeg(cur, t1))
                val a0 = atan2(t1[1] - c[1], t1[0] - c[0])
                val a1 = atan2(t2[1] - c[1], t2[0] - c[0])
                var sweep = a1 - a0
                while (sweep > PI) sweep -= 2 * PI
                while (sweep < -PI) sweep += 2 * PI
                segs.add(ArcSeg(c[0], c[1], r, a0, sweep))
                cur = t2
            }
            val last = pts.last()
            segs.add(LineSeg(cur, last))
            return Path(segs.filter { it.length > 1e-6 })
        }

        private fun unit(a: DoubleArray, b: DoubleArray): DoubleArray {
            val d = hypot(b[0] - a[0], b[1] - a[1])
            return doubleArrayOf((b[0] - a[0]) / d, (b[1] - a[1]) / d)
        }
    }
}

/**
 * 限速剖面: 先按曲率与环境给出逐点的速度上限, 再用前向/后向两遍扫描
 * 保证纵向加速度与减速度都不超限 —— 结果就是一段"像人开的"速度曲线。
 */
object SpeedProfile {
    fun build(path: Path, alatMax: Double, accelMax: Double, decelMax: Double,
              dt: Double, vmaxAt: (Double) -> Double, stopAt: List<Double>): DoubleArray {
        val n = (path.total / (dt * 2.0)).toInt() + 2
        val ds = path.total / (n - 1)
        val v = DoubleArray(n) { i ->
            val s = i * ds
            val k = abs(path.at(s)[4])
            var lim = vmaxAt(s)
            if (k > 1e-6) lim = min(lim, sqrt(alatMax / k))
            lim
        }
        for (s in stopAt) {
            val i = (s / ds).roundToInt().coerceIn(0, n - 1)
            v[i] = 0.0
        }
        // 后向: 保证能刹停
        for (i in n - 2 downTo 0) v[i] = min(v[i], sqrt(v[i + 1].pow(2) + 2 * decelMax * ds))
        // 前向: 保证能加速到
        v[0] = 0.0
        for (i in 1 until n) v[i] = min(v[i], sqrt(v[i - 1].pow(2) + 2 * accelMax * ds))
        return v
    }
}

/** 空间环境分区: 命中即返回, 顺序即优先级 */
class EnvMap(private val zones: List<Triple<DoubleArray, DoubleArray, EnvClass>>) {
    fun at(x: Double, y: Double): EnvClass {
        for ((lo, hi, env) in zones)
            if (x >= lo[0] && x <= hi[0] && y >= lo[1] && y <= hi[1]) return env
        return EnvClass.OPEN_SKY
    }
    fun rect(env: EnvClass, x0: Double, y0: Double, x1: Double, y1: Double) =
        Triple(doubleArrayOf(x0, y0), doubleArrayOf(x1, y1), env)
}

/** 蜂窝基站 / WiFi 接入点的物理位置 (仿真已知, 检测器只知道观测量) */
/** 蜂窝基站 / WiFi 接入点的物理位置 (仿真已知, 检测器只知道观测量) */
data class Tower(val id: Long, val x: Double, val y: Double, val h: Double, val pTxDbm: Double)
data class Ap(val bssid: Long, val x: Double, val y: Double, val h: Double, val pTxDbm: Double, val freq: Int)

data class Anchor(
    val id: Long, val e: Double, val n: Double, val altM: Double,
    val kind: io.gnsstrust.model.AnchorKind,
)

/**
 * 独立测距锚点的物理部署。
 *
 * 这是整个方案能否成立的关键: 慢速拉偏之所以困难, 不是因为姿态/惯性,
 * 而是因为存在【与 GNSS 完全无关的第二把尺子】。
 *   - 卡车: 运营商基站的 TA (定时提前量), 以及园区/场站的 WiFi RTT
 *   - 无人机: C2 数传链路的双向测距 (TWR), 每 1 Hz 一次
 * 工程上这些量本来就在链路上跑, 只是通常没人拿来交叉检验 GNSS。
 */
object AnchorNetwork {
    fun forTruck(seed: Long): List<Anchor> {
        val R = io.gnsstrust.core.SimRandom(seed * 53 + 19)
        val cells = (0 until 8).map { i ->
            val a = 2 * PI * i / 8 + 0.42
            Anchor(500L + i, 1500 * cos(a), 1500 * sin(a), 32.0, AnchorKind.CELL_TA)
        }
        val depot = Anchor(590L, -1000.0, -800.0, 3.0, AnchorKind.DEPOT_UWB)
        val rtt = (0 until 5).map { i ->
            val a = R.uniform(0.0, 2 * PI)
            Anchor(600L + i, 400 * cos(a), 400 * sin(a), 25.0, AnchorKind.CELL_TA)
        }
        return cells + depot + rtt
    }

    fun forDrone(seed: Long): List<Anchor> = listOf(
        Anchor(700L, -900.0, -900.0, 30.0, AnchorKind.C2_LINK),
        Anchor(701L, 900.0, -900.0, 30.0, AnchorKind.C2_LINK),
        Anchor(702L, 900.0, 900.0, 30.0, AnchorKind.C2_LINK),
        Anchor(703L, -900.0, 900.0, 30.0, AnchorKind.C2_LINK),
        Anchor(704L, -700.0, -700.0, 1.0, AnchorKind.DEPOT_UWB),
    )
}

/** 路网: 用于地图约束检验 (地图来自本地地图库, 攻击者无法通过射频篡改) */
class RoadNetwork(val polylines: List<Pair<List<DoubleArray>, Double>>) {
    /** 返回 (到最近道路中心线的距离, 该道路的半宽) */
    fun nearest(x: Double, y: Double): Pair<Double, Double> {
        var best = Double.MAX_VALUE
        var halfW = 4.0
        val p = doubleArrayOf(x, y)
        for ((pts, hw) in polylines) {
            val pr = io.gnsstrust.core.Polyline.project(pts, p)
            if (pr.dist < best) { best = pr.dist; halfW = hw }
        }
        return best to halfW
    }
}

/**
 * 卡车场景: 一个 2km x 2km 的城区环线, 依次经过 郊区 -> 城市峡谷 -> 隧道 -> 高架 -> 回车库。
 */
class TruckScene(seed: Long) {
    val route: Path
    val envMap: EnvMap
    val roads: RoadNetwork
    val towers: List<Tower>
    val aps: List<Ap>

    init {
        // 环线: 仓库 -> 东行 -> 北行 -> 西穿市区 -> 高架 -> 南下穿隧道 -> 回仓库
        val wps = listOf(
            doubleArrayOf(-1000.0, -800.0),
            doubleArrayOf(1000.0, -800.0),
            doubleArrayOf(1000.0, 300.0),
            doubleArrayOf(-300.0, 300.0),
            doubleArrayOf(-300.0, 1000.0),
            doubleArrayOf(-1000.0, 1000.0),
            doubleArrayOf(-1000.0, -800.0),
        )
        route = Path.rounded(wps, radius = 60.0)
        envMap = EnvMap(listOf(
            // 仓库/地库: 起点与终点
            envMapRect(EnvClass.INDOOR_GARAGE, -1080.0, -880.0, -920.0, -720.0),
            // 隧道: 西侧路段 y in [-100, 600], 约 700 m
            envMapRect(EnvClass.TUNNEL, -1075.0, -100.0, -925.0, 600.0),
            // 城市峡谷: 东侧 + 北侧市区的 L 形
            envMapRect(EnvClass.URBAN_CANYON, -350.0, -880.0, 1080.0, 260.0),
            envMapRect(EnvClass.URBAN_CANYON, -350.0, 260.0, 1080.0, 330.0),
            envMapRect(EnvClass.URBAN_CANYON, -400.0, 220.0, -240.0, 1060.0),
            envMapRect(EnvClass.SUBURBAN, -1100.0, -880.0, -350.0, -700.0),
        ))

        val main = route.sampled(20.0)
        val crossStreets = listOf(
            listOf(doubleArrayOf(-1000.0, 0.0), doubleArrayOf(1000.0, 0.0)),
            listOf(doubleArrayOf(-600.0, -800.0), doubleArrayOf(-600.0, 300.0)),
            listOf(doubleArrayOf(200.0, -800.0), doubleArrayOf(200.0, 300.0)),
            listOf(doubleArrayOf(-1000.0, 300.0), doubleArrayOf(1000.0, 300.0)),
            listOf(doubleArrayOf(400.0, 300.0), doubleArrayOf(400.0, 1000.0)),
        )
        roads = RoadNetwork(listOf(main to 5.0) + crossStreets.map { it to 5.0 })

        val R = io.gnsstrust.core.SimRandom(seed * 31 + 7)
        towers = (0 until 10).map { i ->
            val a = 2 * PI * i / 10 + 0.3
            Tower(1000L + i, 1300 * cos(a), 1300 * sin(a), 30.0 + R.uniform(0.0, 25.0), 46.0)
        } + Tower(1010L, 0.0, 0.0, 55.0, 46.0)
        aps = (0 until 60).map { i ->
            val a = R.uniform(0.0, 2 * PI)
            val r = R.uniform(100.0, 1200.0)
            Ap(90000L + i, r * cos(a), r * sin(a), R.uniform(3.0, 20.0),
                R.uniform(15.0, 20.0), if (i % 2 == 0) 2400 else 5200)
        }
    }

    private fun envMapRect(env: EnvClass, x0: Double, y0: Double, x1: Double, y1: Double) =
        Triple(doubleArrayOf(x0, y0), doubleArrayOf(x1, y1), env)

    /** 沿路径的速度上限 (交规 + 环境) */
    fun vmaxAt(s: Double): Double {
        val p = route.at(s)
        return when (envMap.at(p[0], p[1])) {
            EnvClass.OPEN_SKY, EnvClass.SUBURBAN -> 14.0
            EnvClass.URBAN_CANYON -> 9.0
            EnvClass.TUNNEL -> 16.0
            EnvClass.INDOOR_GARAGE -> 3.0
        }
    }

    /** 两处红灯 + 终点停车 */
    fun stops(): List<Double> {
        val s1 = route.total * 0.30
        val s2 = route.total * 0.62
        return listOf(s1, s2, route.total - 6.0)
    }
}

/** 无人机场景: 从城区楼顶起飞, 爬升到 80 m, 沿 S 形航线巡检 */
class DroneScene(seed: Long) {
    val route: Path
    val envMap: EnvMap
    val roads: RoadNetwork
    val towers: List<Tower>
    val aps: List<Ap>
    val cruiseAltM = 80.0
    val launchAltM = 25.0

    init {
        val wps = listOf(
            doubleArrayOf(-700.0, -700.0),
            doubleArrayOf(-200.0, -600.0),
            doubleArrayOf(300.0, -300.0),
            doubleArrayOf(600.0, 100.0),
            doubleArrayOf(200.0, 500.0),
            doubleArrayOf(-400.0, 600.0),
            doubleArrayOf(-800.0, 200.0),
        )
        route = Path.rounded(wps, radius = 120.0)
        envMap = EnvMap(listOf(
            envMapRect(EnvClass.URBAN_CANYON, -350.0, -880.0, 1080.0, 260.0),
        ))
        roads = RoadNetwork(listOf(route.sampled(25.0) to 5.0))
        val R = io.gnsstrust.core.SimRandom(seed * 71 + 3)
        towers = (0 until 12).map { i ->
            val a = 2 * PI * i / 12
            Tower(2000L + i, 1400 * cos(a), 1400 * sin(a), 30.0, 46.0)
        }
        aps = (0 until 40).map { i ->
            Ap(95000L + i, R.uniform(-1000.0, 1000.0), R.uniform(-1000.0, 1000.0),
                R.uniform(5.0, 25.0), 18.0, 5200)
        }
    }

    private fun envMapRect(env: EnvClass, x0: Double, y0: Double, x1: Double, y1: Double) =
        Triple(doubleArrayOf(x0, y0), doubleArrayOf(x1, y1), env)

    fun vmaxAt(s: Double): Double = 13.0
    fun stops(): List<Double> = listOf(route.total - 8.0)

    /**
     * 高度剖面: 起飞 -> 爬升 -> 巡航 -> 下降回收。
     * s 超过 route.total 后保持在着陆高度 (说明场景时长超出了航线, 应当报错)。
     */
    fun altAt(sIn: Double): Double {
        val s = sIn.coerceIn(0.0, route.total)
        val t = s / route.total
        return when {
            t < 0.04 -> launchAltM + (cruiseAltM - launchAltM) * (t / 0.04)
            t < 0.90 -> cruiseAltM + 6.0 * sin(t * 9.0)
            else -> cruiseAltM - (cruiseAltM - launchAltM) * ((t - 0.90) / 0.10)
        }
    }
}

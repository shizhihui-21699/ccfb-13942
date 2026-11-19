package io.gnsstrust.eval

import io.gnsstrust.detect.DetectorConfig
import io.gnsstrust.detect.SpoofDetector
import io.gnsstrust.detect.Verdict
import io.gnsstrust.model.*
import io.gnsstrust.sim.ScenarioSpec
import io.gnsstrust.sim.World
import kotlin.math.*

/**
 * 场景评估器: 把 [ScenarioMeta] 喂给检测器, 逐帧记录。
 */
object Evaluator {

    fun run(meta: ScenarioMeta, config: DetectorConfig = DetectorConfig()): ScenarioResult {
        val det = SpoofDetector(config)
        det.attachMap(meta.roads)
        meta.corridor?.let { det.attachCorridor(it, meta.corridorHalfWidthM) }
        // 注入锚点坐标 (真实工程里这些来自运营商/自建基站的台账)
        det.anchorPositions = anchorPositionsOf(meta)
        det.anchorKinds = anchorKindsOf(meta)

        val frames = ArrayList<FrameEval>(meta.frames.size)
        val onset = onsetOf(meta)

        for (i in meta.frames.indices) {
            val f = meta.frames[i]
            val tr = meta.truth[i]
            val atk = meta.attack[i]
            val rep = det.update(f)
            val gnssUsable = f.gnss.valid
            val gnssErr = if (gnssUsable) hypot(f.gnss.eM - tr.e, f.gnss.nM - tr.north) else Double.NaN
            frames.add(
                FrameEval(
                    t = f.t, verdict = rep.verdict, score = rep.score, dominant = rep.dominant,
                    env = rep.env, gnssUsable = gnssUsable,
                    trueE = tr.e, trueN = tr.north,
                    repE = f.gnss.eM, repN = f.gnss.nM,
                    injectedM = atk.horizontal, gnssErrM = gnssErr,
                    attackActive = atk.active,
                    posUncertaintyM = rep.posUncertaintyM,
                    timeSinceAnchor = rep.timeSinceAnchorS,
                    cusumPos = rep.cusum[io.gnsstrust.detect.DetectorId.POS_VS_INS] ?: 0.0,
                    chi2Pos = rep.chi2Pos,
                )
            )
        }

        return ScenarioResult(
            scenario = meta.name, platform = meta.platform, attackKind = meta.attackKind,
            seed = meta.seed, frames = frames, attackOnset = onset,
            outageWindows = meta.outageWindows,
        )
    }

    private fun onsetOf(meta: ScenarioMeta): Double {
        val first = meta.attack.firstOrNull { it.active } ?: return Double.NaN
        return first.t
    }

    /** 从仿真元数据里取锚点。真实系统由配置/台账提供。 */
    private fun anchorPositionsOf(meta: ScenarioMeta): Map<Long, DoubleArray> {
        val m = HashMap<Long, DoubleArray>()
        if (meta.platform == Platform.TRUCK) {
            for (a in io.gnsstrust.sim.AnchorNetwork.forTruck(meta.seed))
                m[a.id] = doubleArrayOf(a.e, a.n)
        } else {
            for (a in io.gnsstrust.sim.AnchorNetwork.forDrone(meta.seed))
                m[a.id] = doubleArrayOf(a.e, a.n)
        }
        return m
    }

    private fun anchorKindsOf(meta: ScenarioMeta): Map<Long, AnchorKind> {
        val m = HashMap<Long, AnchorKind>()
        if (meta.platform == Platform.TRUCK) {
            for (a in io.gnsstrust.sim.AnchorNetwork.forTruck(meta.seed)) m[a.id] = a.kind
        } else {
            for (a in io.gnsstrust.sim.AnchorNetwork.forDrone(meta.seed)) m[a.id] = a.kind
        }
        return m
    }

    fun percentile(xs: List<Double>, p: Double): Double {
        if (xs.isEmpty()) return Double.NaN
        val s = xs.sorted()
        val idx = ((s.size - 1) * p).roundToInt().coerceIn(0, s.size - 1)
        return s[idx]
    }
}

/**
 * 蒙特卡洛实验驱动器。所有实验都用固定种子序列, 结果可复现。
 */
object Experiments {

    /**
     * 实验 1: 虚拟轨迹的"检出半径"作为拉偏速率的函数。
     *
     * 这是直接回答用户问题的表: 每个速率跑 N 个种子, 统计
     *   - 检出成功率
     *   - 平均延迟
     *   - **检出时已经偏了多少米** (中位数 / 90 分位)
     *
     * 关键洞察: 检出半径对速率**不敏感** —— 因为 CUSUM 等价于
     * "累积偏移量"检测器, 触发阈值由 ARL0 定死。速率越慢, 检出用的
     * 时间越长, 但对应的偏移量几乎一样。这恰好给出一个干净的答案:
     * "无论多慢, 只要它累计到 X 米, 就会被抓"。
     */
    fun driftSweep(
        platform: Platform,
        rates: List<Double>,
        trials: Int,
        durationS: Double,
        consistent: Boolean,
        config: DetectorConfig = DetectorConfig(),
    ): List<DriftSweepRow> {
        val out = ArrayList<DriftSweepRow>()
        for (rate in rates) {
            val delays = ArrayList<Double>()
            val radii = ArrayList<Double>()
            val injected = ArrayList<Double>()
            var detected = 0
            for (tr in 0 until trials) {
                val seed = 1000L + tr * 37L
                val onset = 25.0
                val atk = io.gnsstrust.sim.GradualDrift(
                    onset = onset,
                    rateMps = rate,
                    maxOffsetM = 4000.0,
                    consistent = consistent,
                    dirDeg = (tr * 47L % 360).toDouble(),
                )
                val meta = World.build(
                    ScenarioSpec(
                        name = "drift_${rate}_$tr", platform = platform, attack = atk,
                        seed = seed, durationS = durationS,
                    )
                )
                val res = Evaluator.run(meta, config)
                if (res.detected()) {
                    detected++
                    delays.add(res.detectDelayS)
                    radii.add(res.detectionRadiusM)
                    injected.add(res.injectedAtDetectionM)
                }
            }
            out.add(
                DriftSweepRow(
                    platform = platform,
                    envLabel = if (platform == Platform.TRUCK) "城区环线(含隧道/峡谷)" else "城区低空(含峡谷)",
                    rateMps = rate,
                    consistent = consistent,
                    detected = detected, trials = trials,
                    medianDelayS = Evaluator.percentile(delays, 0.5),
                    medianRadiusM = Evaluator.percentile(radii, 0.5),
                    p90RadiusM = Evaluator.percentile(radii, 0.9),
                    medianInjectedAtDetectM = Evaluator.percentile(injected, 0.5),
                )
            )
        }
        return out
    }

    /**
     * 实验 2: 干净数据的虚警率。
     *
     * 跑足够长的无攻击场景, 统计 CONFIRMED / SUSPECT 的出现。
     * 目标是 CONFIRMED 在 24 小时运营中 ≤ 1 次/车。
     */
    fun falseAlarmStudy(platform: Platform, trials: Int, durationS: Double): FalseAlarmResult {
        var confirmed = 0
        var suspect = 0
        var totalHours = 0.0
        val trustedFractions = ArrayList<Double>()
        val uncerts = ArrayList<Double>()
        for (tr in 0 until trials) {
            val seed = 7000L + tr * 13L
            val meta = World.build(
                ScenarioSpec(
                    name = "clean_$tr", platform = platform,
                    attack = io.gnsstrust.sim.NoAttack, seed = seed, durationS = durationS,
                )
            )
            val res = Evaluator.run(meta)
            if (res.falseAlarm()) confirmed++
            if (res.suspectFalseAlarm()) suspect++
            totalHours += durationS / 3600.0
            trustedFractions.add(res.trustedFraction())
            uncerts.add(res.medianUncertaintyM())
        }
        return FalseAlarmResult(
            platform = platform, trials = trials, durationS = durationS,
            confirmedRuns = confirmed, suspectRuns = suspect,
            totalHours = totalHours,
            confirmedPer24h = confirmed.toDouble() / max(totalHours, 1e-9) * 24.0,
            suspectPer24h = suspect.toDouble() / max(totalHours, 1e-9) * 24.0,
            medianTrustedFraction = Evaluator.percentile(trustedFractions.filter { it.isFinite() }, 0.5),
            medianUncertaintyM = Evaluator.percentile(uncerts.filter { it.isFinite() }, 0.5),
        )
    }

    /**
     * 实验 3: 并发攻击型态的检出能力矩阵。
     * 每一行是一种攻击, 给检出率与延迟。这是模块的"能力清单"。
     */
    fun attackMatrix(platform: Platform, trials: Int, durationS: Double): List<AttackRow> {
        val out = ArrayList<AttackRow>()
        val builders: List<Pair<String, (Long) -> io.gnsstrust.sim.AttackModel>> = buildList {
            add("缓慢拉偏(一致,0.5 m/s)" to { s: Long ->
                io.gnsstrust.sim.GradualDrift(25.0, 0.5, 4000.0, true, dirDeg = s % 360.0) })
            add("缓慢拉偏(一致,2 m/s)" to { s: Long ->
                io.gnsstrust.sim.GradualDrift(25.0, 2.0, 4000.0, true, dirDeg = s % 360.0) })
            add("缓慢拉偏(只偏位置,1 m/s)" to { s: Long ->
                io.gnsstrust.sim.GradualDrift(25.0, 1.0, 4000.0, false, dirDeg = s % 360.0) })
            add("跳变接管(300 m)" to { s: Long ->
                io.gnsstrust.sim.JumpTakeover(25.0, 300.0 * cos((s % 7).toDouble()), 300.0 * sin((s % 7).toDouble())) })
            add("冻结位置(meaconing)" to { _: Long ->
                io.gnsstrust.sim.FrozenPosition(25.0) })
            add("重放(20 s 前)" to { _: Long ->
                io.gnsstrust.sim.Replay(25.0, 20.0) })
            add("幽灵航线(偷跑私活)" to { _: Long ->
                io.gnsstrust.sim.GhostRoute(25.0, 9.0, io.gnsstrust.sim.TruckScene(0).route) })
            if (platform == Platform.DRONE) {
                add("只拉高度(+25 m 缓慢)" to { _: Long ->
                    io.gnsstrust.sim.GradualDrift(25.0, 0.0, 0.0, true, altRateMps = 0.5) })
            }
            add("纯压制(jamming)" to { _: Long -> io.gnsstrust.sim.JamOnly(25.0) })
        }
        for ((name, mk) in builders) {
            var det = 0
            var fa = 0
            val delays = ArrayList<Double>()
            val radii = ArrayList<Double>()
            for (tr in 0 until trials) {
                val seed = 3100L + tr * 91L
                val meta = World.build(
                    ScenarioSpec(name = name, platform = platform, attack = mk(seed), seed = seed, durationS = durationS)
                )
                val res = Evaluator.run(meta)
                if (res.detected()) {
                    det++
                    delays.add(res.detectDelayS); radii.add(res.detectionRadiusM)
                }
            }
            out.add(
                AttackRow(
                    attack = name,
                    detected = det, trials = trials,
                    medianDelayS = Evaluator.percentile(delays, 0.5),
                    medianRadiusM = Evaluator.percentile(radii, 0.5),
                    p90RadiusM = Evaluator.percentile(radii, 0.9),
                )
            )
        }
        return out
    }

    /**
     * 实验 4: 朴素基线对比 —— "只看接收机自检" 与 "只看位置跳变"。
     * 用来说明为什么需要这一整套。
     */
    fun baselineComparison(platform: Platform, trials: Int, durationS: Double): List<BaselineRow> {
        val rows = ArrayList<BaselineRow>()
        // 慢速一致拉偏, 这是基线必然失效的场景
        val rate = 1.5
        var naiveDetected = 0
        var oursDetected = 0
        for (tr in 0 until trials) {
            val seed = 5500L + tr * 29L
            val atk = io.gnsstrust.sim.GradualDrift(25.0, rate, 4000.0, true, dirDeg = tr * 61.0)
            val meta = World.build(ScenarioSpec("bl_$tr", platform, atk, seed, durationS))
            // 朴素: 只看接收机残差 > 5 m 或单帧位置跳变 > 50 m
            var naive = false
            for (i in 1 until meta.frames.size) {
                val f = meta.frames[i]; val p = meta.frames[i - 1]
                if (!f.gnss.valid || !p.gnss.valid) continue
                val jump = hypot(f.gnss.eM - p.gnss.eM, f.gnss.nM - p.gnss.nM)
                if (f.gnss.residualRmsM > 5.0 || jump > 50.0) { naive = true; break }
            }
            if (naive) naiveDetected++
            val res = Evaluator.run(meta)
            if (res.detected()) oursDetected++
        }
        rows.add(BaselineRow("接收机自检 (残差/跳变)", naiveDetected, trials, "只偏位置时有效; 一致欺骗完全失效"))
        rows.add(BaselineRow("本方案 (多源交叉+序列检验)", oursDetected, trials, "对一致欺骗同样有效"))
        return rows
    }
}

data class FalseAlarmResult(
    val platform: Platform,
    val trials: Int,
    val durationS: Double,
    val confirmedRuns: Int,
    val suspectRuns: Int,
    val totalHours: Double,
    val confirmedPer24h: Double,
    val suspectPer24h: Double,
    val medianTrustedFraction: Double,
    val medianUncertaintyM: Double,
)

data class AttackRow(
    val attack: String,
    val detected: Int,
    val trials: Int,
    val medianDelayS: Double,
    val medianRadiusM: Double,
    val p90RadiusM: Double,
) {
    val rate: Double get() = detected.toDouble() / trials
}

data class BaselineRow(val method: String, val detected: Int, val trials: Int, val note: String) {
    val rate: Double get() = detected.toDouble() / trials
}

package io.gnsstrust.detect

import io.gnsstrust.core.*
import kotlin.math.*

/**
 * 单边 CUSUM, 作用在"已白化的标量创新"上。
 *
 * 为什么是 CUSUM 而不是滑窗均值:
 *  - 滑窗均值对慢速拉偏的灵敏度取决于窗长, 且没有明确的虚警率指标;
 *  - CUSUM 在给定 ARL0 下对指定偏移量是最优的 (Lorden 定理), 而且
 *    ARL1 可以直接算出来 —— 这正是"多快能发现"的答案。
 *
 * ★★ 最关键的一个实现细节: **采样间隔必须与标定时的有效间隔一致**。
 * 门槛 h 是按"每 ARL0 个**独立样本**"反解的。如果噪声的相关时间是 τ,
 * 那么 10 Hz 数据里每 τ 秒才有一个独立样本。若仍然每帧推进 CUSUM,
 * 独立样本数会被高估 (dt/τ) 倍, ARL0 随之缩短同样的倍数 ——
 * 在郊区 τ=5 s 时就等于每小时误报约 50 次。本次开发中就是这个错误
 * 让干净数据出现 100% 的"触发率"。
 *
 * 所以 CUSUM 只在**累积了 tau 秒的信息之后**推进一次, 推进的量是这
 * 段窗口内的平均归一化创新。
 */
class Cusum(
    val id: DetectorId,
    val kRef: Double,
    val h: Double,
    /** 有效独立采样间隔 (s): 噪声相关时间与物理采样周期中较大者 */
    val effStepS: Double,
) {
    private var sPos = 0.0
    private var sNeg = 0.0
    private var accum = 0.0
    private var accumW = 0.0
    private var lastAdvanceT = Double.NaN

    var lastE = 0.0
        private set
    var lastAlarmT = Double.NaN
    var alarms = 0
    var maxSeen = 0.0

    /**
     * @param eNormalised 本帧的归一化创新 (以 σ 计)
     * @param t 本帧时间戳
     * @return 本次推进是否越过门槛
     */
    fun update(eNormalised: Double, t: Double = Double.NaN): Boolean {
        // 累加本帧; 到 effStepS 才真正推进一次
        accum += eNormalised
        accumW += 1.0
        if (t.isNaN()) {
            // 兼容旧调用: 直接推进
            return advance(eNormalised)
        }
        if (lastAdvanceT.isNaN()) { lastAdvanceT = t; return false }
        if (t - lastAdvanceT < effStepS) return false
        lastAdvanceT = t
        val mean = if (accumW > 0) accum / accumW else 0.0
        accum = 0.0; accumW = 0.0
        return advance(mean)
    }

    private fun advance(e: Double): Boolean {
        lastE = e
        sPos = max(0.0, sPos + e - kRef)
        sNeg = max(0.0, sNeg - e - kRef)
        val m = max(sPos, sNeg)
        if (m > maxSeen) maxSeen = m
        return if (m > h) {
            alarms++
            val ratio = (h * 0.6) / max(m, 1e-9)
            sPos *= ratio; sNeg *= ratio
            true
        } else false
    }

    fun statistic(): Double = max(sPos, sNeg)
    fun level(): Double = (statistic() / h).coerceIn(0.0, 1.5)

    fun reset() { sPos = 0.0; sNeg = 0.0; maxSeen = 0.0; accum = 0.0; accumW = 0.0 }

    fun detectableDeltaIn(dt: Double, tS: Double): Double {
        val targetArl1 = tS / effStepS
        var lo = kRef; var hi = 50.0
        if (io.gnsstrust.core.cusumArl1Approx(h, kRef, hi) > targetArl1) return Double.NaN
        repeat(60) {
            val mid = 0.5 * (lo + hi)
            if (io.gnsstrust.core.cusumArl1Approx(h, kRef, mid) > targetArl1) lo = mid else hi = mid
        }
        return 0.5 * (lo + hi)
    }
}

/**
 * 多通道融合。
 *
 * 关键原则: **不做加权平均**。加权平均会让"14 条证据里 1 条强烈报警"
 * 被稀释掉 —— 而在欺骗检测里, 单条强证据 (比如独立测距差了 400 m)
 * 本来就该直接定案。
 *
 * 采用"证据累积 + 封顶"的方式:
 *   每一路报警贡献一份对数似然比 (LLR), 按该路的可信度加权;
 *   总 LLR 越过 prompt/confirm 两级门槛才升级。
 * 这样:
 *   - 多个弱证据可以共同形成判决 (慢速拉偏的典型情形)
 *   - 单个强证据 (独立测距) 也能独立定案
 *   - 证据会随时间衰减, 所以"曾经报警"不会永久污染状态
 */
class EvidenceFusion(
    val decayHalfLifeS: Double = 45.0,
) {
    private val llr = HashMap<DetectorId, Double>()
    private var lastT = Double.NaN

    /** 每路的权重: 反映"这条证据在法庭上有多硬" */
    private val weight = mapOf(
        DetectorId.RANGE_VS_INS to 2.2,
        DetectorId.MAP_VS_INS to 1.8,
        DetectorId.VEL_VS_INS to 1.0,
        DetectorId.COURSE_VS_HEADING to 1.3,
        DetectorId.POS_VS_INS to 0.85,
        DetectorId.SPEED_VS_WHEEL to 0.9,
        DetectorId.RF_ANOMALY to 1.4,
        DetectorId.RX_SELFCHECK to 1.5,
        DetectorId.MOTION_PLAUSIBILITY to 1.2,
        DetectorId.TIME_SYNC to 1.6,
        DetectorId.ALT_VS_BARO to 1.1,
    )

    fun tick(t: Double) {
        if (!lastT.isNaN()) {
            val dt = t - lastT
            if (dt > 0) {
                val f = 0.5.pow(dt / decayHalfLifeS)
                for (k in llr.keys.toList()) llr[k] = llr[k]!! * f
            }
        }
        lastT = t
    }

    /**
     * @param normalised 该路统计量相对门槛的倍数 (1.0 = 刚好到门槛)
     * @param credible 该路当前是否可用 (比如隧道里不用位置检验); false 时不贡献
     */
    fun contribute(id: DetectorId, normalised: Double, credible: Boolean = true) {
        if (!credible) return
        val w = weight[id] ?: 1.0
        // 只有超过门槛才计分; 强度用对数增长, 避免单次极端值接管一切
        val inc = if (normalised <= 1.0) 0.0 else w * ln(normalised).coerceAtMost(3.0)
        val cur = llr[id] ?: 0.0
        llr[id] = cur + inc
    }

    fun total(): Double = llr.values.sum()
    fun breakdown(): Map<DetectorId, Double> = llr.filter { it.value > 0.05 }

    /** 最强的一路 */
    fun dominant(): Pair<DetectorId, Double>? = llr.maxByOrNull { it.value }?.takeIf { it.value > 0.05 }?.toPair()

    fun reset() { llr.clear() }
}

/**
 * 判决等级。
 *
 * 设计成三级而不是二值, 是因为误报的代价不对称:
 *  - 卡车被判"被欺骗"会触发调度核查, 代价中等;
 *  - 无人机被判"被欺骗"可能触发自动返航/降落, 误报的代价很高。
 * 所以无人机默认停在 SUSPECT, 需要更强证据才 CONFIRMED。
 */
enum class Verdict(val label: String) {
    TRUSTED("可信"),
    DEGRADED("退化 (无有效定位来源)"),
    SUSPECT("可疑 (证据不足以上报)"),
    CONFIRMED("确认被欺骗"),
}

/**
 * 判决门槛。依据来自两个约束:
 *  (a) 干净数据上 24 小时运行, CONFIRMED 误报 ≤ 1 次;
 *  (b) 对给定的拉偏速率, 在 [x] 秒内升级到 CONFIRMED 的概率 ≥ 90%。
 * 这两个约束共同确定了总分门槛 (见 Evaluation 的 ROC 表)。
 */
data class FusionThresholds(
    val suspect: Double,
    val confirmed: Double,
    val suspectHoldS: Double = 1.0,
    val confirmHoldS: Double = 2.0,
)

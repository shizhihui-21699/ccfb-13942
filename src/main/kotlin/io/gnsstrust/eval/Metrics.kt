package io.gnsstrust.eval

import io.gnsstrust.detect.DetectorId
import io.gnsstrust.detect.Verdict
import io.gnsstrust.model.*

/**
 * 把场景跑过检测器, 逐帧记账。
 *
 * 评估必须回答的三个问题:
 *  1. 干净数据上会不会误报 (虚警率 / 每天次数)
 *  2. 攻击发生到被发现, 用了多久、那时已经被拉偏多远 (检测延迟 / 检出半径)
 *  3. 判错的代价: 漏报 (攻击持续但一直 TRUSTED) 和误报各多少
 */
data class FrameEval(
    val t: Double,
    val verdict: Verdict,
    val score: Double,
    val dominant: DetectorId?,
    val env: EnvClass,
    val gnssUsable: Boolean,
    /** 真实位置 */
    val trueE: Double, val trueN: Double,
    /** GNSS 报告位置 */
    val repE: Double, val repN: Double,
    /** 攻击注入的偏移量 */
    val injectedM: Double,
    /** GNSS 与真值的实际偏差 (含自然误差) */
    val gnssErrM: Double,
    /** 攻击者声称的偏移 (攻击模型认为它拉了多少) */
    val attackActive: Boolean,
    val posUncertaintyM: Double,
    val timeSinceAnchor: Double,
    val cusumPos: Double,
    val chi2Pos: Double,
)

data class ScenarioResult(
    val scenario: String,
    val platform: Platform,
    val attackKind: String,
    val seed: Long,
    val frames: List<FrameEval>,
    val attackOnset: Double,
    val outageWindows: List<ClosedFloatingPointRange<Double>>,
) {
    /** 第一次 CONFIRMED 的时刻; NaN 表示从未 */
    val firstConfirmed: Double get() = frames.firstOrNull { it.verdict == Verdict.CONFIRMED }?.t ?: Double.NaN
    val firstSuspect: Double get() = frames.firstOrNull {
        it.verdict == Verdict.SUSPECT || it.verdict == Verdict.CONFIRMED
    }?.t ?: Double.NaN

    /** 检测延迟 (相对攻击开始), 只对攻击场景有意义 */
    val detectDelayS: Double get() {
        val fc = firstConfirmed
        return if (attackOnset.isNaN() || fc.isNaN()) Double.NaN else fc - attackOnset
    }

    val suspectDelayS: Double get() {
        val fs = firstSuspect
        return if (attackOnset.isNaN() || fs.isNaN()) Double.NaN else fs - attackOnset
    }

    /**
     * **检出半径**: 第一次确认被欺骗时, GNSS 已经偏离真实位置多少米。
     * 这是本题最核心的输出 —— "被拉偏多远时能发现"。
     */
    val detectionRadiusM: Double get() {
        val fc = firstConfirmed
        if (fc.isNaN()) return Double.NaN
        val fr = frames.firstOrNull { it.t >= fc } ?: return Double.NaN
        return fr.gnssErrM
    }

    /** 第一次 CONFIRMED 时攻击模型注入的偏移 (理想情况下它已完全生效) */
    val injectedAtDetectionM: Double get() {
        val fc = firstConfirmed
        if (fc.isNaN()) return Double.NaN
        return frames.firstOrNull { it.t >= fc }?.injectedM ?: Double.NaN
    }

    /** 攻击是否被检出 (在攻击窗口内有 CONFIRMED) */
    fun detected(): Boolean {
        if (attackOnset.isNaN()) return false
        return frames.any { it.t >= attackOnset && it.verdict == Verdict.CONFIRMED }
    }

    /** 干净场景里是否有 CONFIRMED (误报) */
    fun falseAlarm(): Boolean = attackOnset.isNaN() && frames.any { it.verdict == Verdict.CONFIRMED }

    fun suspectFalseAlarm(): Boolean =
        attackOnset.isNaN() && frames.any { it.verdict == Verdict.SUSPECT || it.verdict == Verdict.CONFIRMED }

    /** 只在 GNSS 可用且不在失锁窗口里的帧上统计 */
    fun trustedFraction(): Double {
        val usable = frames.filter { it.gnssUsable }
        if (usable.isEmpty()) return Double.NaN
        return usable.count { it.verdict == Verdict.TRUSTED }.toDouble() / usable.size
    }

    /** 稳态下的位置不确定度 (中位数) */
    fun medianUncertaintyM(): Double {
        val v = frames.filter { it.gnssUsable && it.posUncertaintyM.isFinite() }
            .map { it.posUncertaintyM }.sorted()
        return if (v.isEmpty()) Double.NaN else v[v.size / 2]
    }

    /** 漏报率: 攻击生效 (注入 > 25 m) 但仍判 TRUSTED 的帧占比 */
    fun missRate(): Double {
        val exposed = frames.filter { it.attackActive && it.injectedM > 25.0 && it.gnssUsable }
        if (exposed.isEmpty()) return Double.NaN
        return exposed.count { it.verdict == Verdict.TRUSTED }.toDouble() / exposed.size
    }
}

/**
 * 一次"缓慢拉偏"实验的完整结果: 扫描速率 vs 检出半径。
 * 这是直接回答"被拉偏多远时能发现"的表格。
 */
data class DriftSweepRow(
    val platform: Platform,
    val envLabel: String,
    val rateMps: Double,
    val consistent: Boolean,
    val detected: Int,
    val trials: Int,
    val medianDelayS: Double,
    val medianRadiusM: Double,
    val p90RadiusM: Double,
    val medianInjectedAtDetectM: Double,
) {
    val detectRate: Double get() = detected.toDouble() / trials
    val rateMperMin: Double get() = rateMps * 60.0
}

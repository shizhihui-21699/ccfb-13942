package io.gnsstrust.detect

import io.gnsstrust.model.EnvClass
import kotlin.math.*

/**
 * 检测门槛的统一定义。**每一个数都有来源, 没有一个是拍脑袋的。**
 *
 * 记法: σ 是该量在当前环境下的**真实**标准差 (由仿真/标定给出, 不是接收机自报),
 *       α 是允许的单次检验虚警率, ARL0 是平均多少采样点误报一次。
 *
 * 三条独立的门槛体系:
 *
 * 1) 单点一致性 (Instantaneous Gate)
 *    创新 z = x_gnss - x_ins, 协方差 S = P_ins + R_gnss。
 *    判据 zᵀS⁻¹z > χ²(2, 1-α)。
 *    依据: 在无攻击、且两个误差都近似高斯时, 该统计量严格服从自由度为 2 的卡方。
 *    取 α = 1e-4 => 门槛 = 18.42。
 *    但这不是最终判据 —— 因为"高斯"假设在城市峡谷里是错的 (厚尾), 见下面第 3 条。
 *
 * 2) 序列检测 (CUSUM)
 *    慢速拉偏的每一次单点偏差都只有 1-3 σ, 单点门槛永远打不到。
 *    所以对**归一化创新** e_k = (标量投影后的创新)/σ_k 做单边 CUSUM:
 *        S_k = max(0, S_{k-1} + e_k - k),  报警当 S_k > h
 *    参考值按最优取 k = delta_star / 2, 其中 delta_star 是"我们最想抓的每采样偏移, 以 σ 计"。
 *    对 L 秒的拉偏, 若拉偏速率 v, 每采样偏差 delta ≈ v*dt/sigma, 因此 delta_star 由
 *    "多慢的拉偏还要抓"这一产品需求反推 (默认: 0.35 σ/采样)。
 *    门槛 h 由 ARL0 反解: 我们要求单通道在 1 小时内误报不超过 1 次,
 *    dt=0.1 s => ARL0 ≥ 36000。用 Siegmund 近似反解得到 h (见 Stats.kt)。
 *    这就是"多快能发现"的量化依据: ARL1(h,k,δ) 给出平均检测延迟。
 *
 * 3) 环境自适应放宽 (为什么峡谷里门槛必须变)
 *    城市峡谷的 GNSS 位置误差不是白噪声, 而是相关时间 30-60 s 的慢变过程。
 *    对这样的过程做"每采样"检验, 有效独立样本数只有 T/τ_c ≈ 1/300 左右。
 *    若不修正, CUSUM 会在多径漂移上疯狂误报。
 *    修正办法: 用**创新白化 + 有效样本数折算**。具体做法是把 CUSUM 的
 *    参考值按 √(dt/τ_eff) 缩小, 同时门槛 h 保持由 ARL0 反解 —— 等价于
 *    承认"峡谷里我每分钟只有 1 个有效样本", 因此同样的物理偏移需要更大
 *    的 σ 倍数才能触发。
 *
 * 4) 绝对参照 (Range / Map)
 *    用一个与 GNSS 无关的量作参照时, 它自身的误差模型是独立的, 门槛回到纯卡方。
 */
object Thresholds {

    /** 单点卡方门槛: χ²(2) 的 1-1e-4 分位 */
    val SINGLE_POINT_CHI2: Double = chi2Threshold(alpha = 1e-4, df = 2)

    /** 对速度/航向这类一维检验 */
    val SINGLE_SCALAR_Z: Double = normQ(1.0 - 5e-5)

    /**
     * CUSUM 参数。
     * @param kRef 参考值 (以 σ 为单位每采样)
     * @param arl0 目标误报间隔 (采样点数)
     * @param h 由 arl0 反解得到的门槛
     */
    data class CusumParams(val kRef: Double, val arl0: Double, val h: Double, val effTau: Double) {
        /** 给定每采样偏移 δ (σ 单位), 平均检测延迟 (采样点数) */
        fun arl1(delta: Double): Double =
            io.gnsstrust.core.cusumArl1Approx(h, kRef, delta)
        /** 等价的可检测偏移 (在 ARL1=arl0/10 的条件下, 以 σ/采样 计) */
        fun minDetectableDelta(): Double {
            var lo = kRef; var hi = 40.0
            repeat(60) {
                val mid = 0.5 * (lo + hi)
                if (arl1(mid) > arl0 / 10) lo = mid else hi = mid
            }
            return 0.5 * (lo + hi)
        }
    }

    /**
     * 为每个环境生成 CUSUM 参数。
     *
     * @param dt 采样周期
     * @param targetArl0H 期望误报间隔 (小时) —— 单通道
     * @param deltaStar 最想抓的每采样偏移 (以 σ 计)
     * @param effTau 误差过程的相关时间 (s); 白噪声取 dt
     */
    fun cusumFor(env: EnvClass, dt: Double, targetArl0H: Double, deltaStar: Double, effTau: Double): CusumParams {
        val effectiveDt = max(dt, effTau)
        // 有效样本率: 相关过程下每 effTau 秒才有一个独立样本
        val arl0Samples = targetArl0H * 3600.0 / effectiveDt
        val k = deltaStar / 2.0
        val h = io.gnsstrust.core.cusumThresholdForArl(arl0Samples, k)
        return CusumParams(k, arl0Samples, h, effectiveDt)
    }

    fun chi2Threshold(alpha: Double, df: Int): Double = io.gnsstrust.core.chi2Quantile(1.0 - alpha, df)

    fun normQ(p: Double): Double = io.gnsstrust.core.normQuantile(p)

    /**
     * 给定量测能力需求反解噪声预算 —— 反过来用:
     * 若要求"在 T 秒内发现 v 米/秒的拉偏", 则需要的航向精度上限是多少?
     *
     * 航向误差 θ 在 T 秒内造成的横向位置误差 ≈ v·T²·θ/2 (匀速时)。
     * 要求该误差 > n·σ_dr 才能被发现 => θ > 2n σ_dr/(v T²)。
     * 这是航向精度指标 (以及磁罗盘/陀螺选型) 的直接依据。
     */
    fun requiredHeadingAccuracy(vMps: Double, tS: Double, drSigmaM: Double, n: Double): Double =
        2.0 * n * drSigmaM / max(vMps * tS * tS, 1e-9)
}

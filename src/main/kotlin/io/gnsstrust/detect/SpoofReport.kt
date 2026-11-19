package io.gnsstrust.detect

import io.gnsstrust.model.*

/** 一帧的判决结果 —— 这是模块对外的全部契约 */
data class SpoofReport(
    val t: Double,
    val verdict: Verdict,
    /** 0..1, 判决的置信度 (证据充分性, 不是概率) */
    val confidence: Double,
    /** 融合评分 */
    val score: Double,
    /** 最强的一路证据 */
    val dominant: DetectorId?,
    /** 人可读的理由, 供调度/审计 */
    val reasons: List<String>,
    /** 惯导自身的位置不确定度 (m) */
    val navPosSigmaM: Double,
    /** 综合考虑 GNSS 与环境后的位置不确定度 (m); NaN 表示完全不可知 */
    val posUncertaintyM: Double,
    /** 距离上次接受 GNSS 重锚的时间 (s) */
    val timeSinceAnchorS: Double,
    val gnssUsable: Boolean,
    val env: EnvClass,
    /** 各路检验的归一化强度 (1.0 = 刚好到门槛) */
    val measurements: Map<DetectorId, Double>,
    /** CUSUM 当前的充实度 (0..1.5) */
    val cusum: Map<DetectorId, Double>,
    val chi2Pos: Double,
    val chi2Vel: Double,
    /** 内部状态, 便于诊断: BOOTSTRAP / TRACKING / LOST */
    val phase: String = "",
)

/**
 * 检测器配置。所有可调的旋钮集中在这里, 且每一项都配了"为什么是这个值"。
 */
data class DetectorConfig(
    // ---------------- 序列检验 ----------------
    /** 单通道目标误报间隔 (小时)。取 1 h: 车队 200 台车 => 全网约 12 分钟一次误报, 可接受 */
    val arl0Hours: Double = 1.0,
    /**
     * 最想抓的"每采样偏移", 以 σ 计。
     *
     * 依据: 高速公路场景下, 0.5 m/s 的拉偏 (1800 m/h) 已经足以把车
     * 引到完全错误的位置, 而 GPS 位置 σ≈2.5 m, 采样 10 Hz =>
     * 每采样偏移 δ = 0.5*0.1/2.5 = 0.02 σ。以 0.35 σ 为参考值意味着
     * 我们优先抓"每 3 秒偏移 1 σ"以上量级, 也就是 25 m/min 的拉偏。
     * 更慢的拉偏在物理上无法与惯导自身漂移区分 (见 README 的不可能性论证)。
     */
    val deltaStar: Double = 0.35,

    /** 误差相关时间 (s) —— 决定有效独立样本数 */
    val envTau: Map<EnvClass, Double> = mapOf(
        EnvClass.OPEN_SKY to 2.0,
        EnvClass.SUBURBAN to 5.0,
        EnvClass.URBAN_CANYON to 45.0,
        EnvClass.TUNNEL to 60.0,
        EnvClass.INDOOR_GARAGE to 60.0,
    ),

    // ---------------- 重锚条件 ----------------
    /** 惯导可以漂多久仍用于比对 (s) */
    val maxDrForGateS: Double = 120.0,
    /** 惯导位置不确定度超过此值就不再用于比对 (m) */
    val maxDrSigmaForGate: Double = 45.0,
    val minSpeedForCourseMps: Double = 2.0,
    /**
     * 无人机是否有独立航向源 (磁罗盘/视觉里程计/C2 来向)。
     * 现实中多数小型无人机没有 —— 此时航向类检验必须关闭。
     */
    val droneHasHeadingSource: Boolean = false,

    // ---------------- 融合门槛 ----------------
    val thresholds: FusionThresholds = FusionThresholds(
        suspect = 2.2,
        confirmed = 5.5,
        suspectHoldS = 1.5,
        confirmHoldS = 2.5,
    ),

    val fusionDecayHalfLifeS: Double = 45.0,

    /** 独立测距的门槛 (单次), 以 σ 计 */
    val rangeGate: Double = 4.0,
    /**
     * 惯导必须自由运行至少这么久, 独立测距的残差才算独立证据。
     * 取 3 s: 短于此, 惯导位置里还残留着上一次重锚注入的 GNSS 噪声,
     * 残差会自相关。
     */
    val rangeFreeRunS: Double = 3.0,
    /** 射频指纹的门槛 */
    val rfGate: Double = 3.0,

    // ---------------- 状态机 ----------------
    val dt: Double = 0.1,
    /**
     * 冷启动要连续多少帧"零异常"才允许建立第一个绝对锚点。
     *
     * 依据: 默认门限下, 干净数据每帧误报概率 < 1e-3, 连续 50 帧 (5 s)
     * 全都干净的概率 > 0.95; 而攻击者要让一段假轨迹在射频指纹、接收机自检、
     * 运动学、地图四项上全部过关, 代价远高于等待 5 s。
     */
    val bootstrapCleanFrames: Int = 50,
    /** 失锁超过这么久, 恢复后必须重新 bootstrap (惯导已经漂到不可信) */
    val lostReacquireS: Double = 45.0,
    /** 失锁恢复后的门槛爬升时间 */
    val recoveryRampS: Double = 20.0,

    /** 单点卡方的安全系数: 1.0 = 严格按 α=1e-4 */
    val chi2SafetyFactor: Double = 1.0,
    /** 环境分类置信度低时整体放宽 */
    val envUncertaintyRelax: Double = 2.0,

    /**
     * 时间一致性检验的窗口长度 (s)。
     *
     * 依据: 位置白噪声 σ_p≈4 m (峡谷), 窗口位移的标准差 ≈ √2·σ_p ≈ 5.7 m。
     * 若拉偏速率 1 m/s, 窗口 2 s 内注入偏移的变化 ≈ 2 m —— 还不够。
     * 若拉偏 3 m/s, 2 s 内注入偏移变化 6 m, 与噪声同量级, 开始可检。
     * 窗口越长信噪比越好 (∝ v·W/(√2 σ_p)), 但窗口太长会让"拉偏速率变化"
     * 和"惯导航向漂移"混进来。2 s 是工程折中, 与 10 Hz 的噪声自相关时间
     * (≈1 Hz 以下的慢变) 错开。
     */
    val velocityWindowS: Double = 2.0,
) {
    fun chi2Safety(): Double = chi2SafetyFactor

    fun effTau(env: EnvClass): Double = envTau[env] ?: 5.0

    /**
     * 多径相关误差的预算 (m)。这一项决定"峡谷里能容忍多大位置差"。
     * 取值来自实测经验: 开阔 1 m, 郊区 3 m, 峡谷 12 m, 且随时间略有增长
     * (多径的相关时间约 45 s, 超过后就近似独立)。
     */
    fun envCorrelatedSigma(env: EnvClass, sinceAnchor: Double = 0.0): Double {
        val base = when (env) {
            EnvClass.OPEN_SKY -> 1.0
            EnvClass.SUBURBAN -> 3.0
            EnvClass.URBAN_CANYON -> 12.0
            EnvClass.TUNNEL -> 25.0
            EnvClass.INDOOR_GARAGE -> 30.0
        }
        return base
    }

    fun velSigma(env: EnvClass, sinceAnchor: Double): Double = when (env) {
        EnvClass.OPEN_SKY -> 0.15
        EnvClass.SUBURBAN -> 0.25
        EnvClass.URBAN_CANYON -> 0.45
        EnvClass.TUNNEL -> 0.8
        EnvClass.INDOOR_GARAGE -> 0.8
    }
}

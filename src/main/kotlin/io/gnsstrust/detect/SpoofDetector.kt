package io.gnsstrust.detect

import io.gnsstrust.core.*
import io.gnsstrust.est.NavConfig
import io.gnsstrust.est.NavEstimator
import io.gnsstrust.model.*
import kotlin.math.*

/**
 * 定位欺骗检测器。
 *
 * 每 100 ms 吃一帧 [SensorFrame], 输出一份 [SpoofReport]。
 *
 *  传感器 ──┬─> EnvClassifier ─────────────> 噪声模型 (σ 随环境变)
 *          ├─> NavEstimator (IMU+轮速+磁+气压+光流, **绝不含 GNSS**)
 *          └─> 各路独立检验 ──> 归一化统计量 ──> CUSUM/卡方 ──> 证据融合 ──> 判决
 *
 * 三个状态:
 *
 *   BOOTSTRAP  还没有可信锚点。只做"不需要绝对位置"的检验
 *              (接收机自检、射频指纹、运动学、地图、高度)。
 *              一旦干净地通过一段时间, 才用 GNSS 建立第一个锚点。
 *              这段时间里输出 DEGRADED —— 因为此刻我们**确实**不知道
 *              自己在哪, 也不该假装知道。冷启动骗局就死在这里。
 *
 *   TRACKING   已有锚点, 惯导在漂但漂得可测。全部检验生效。
 *
 *   LOST       GNSS 不可用 (隧道/地库/被压制)。只靠惯导, 输出 DEGRADED,
 *              位置不确定度随失锁时长增长。绝不产生位置类证据。
 *
 * 三条设计红线:
 *  1. GNSS 永不喂进估值器 —— 否则估值器会跟着假轨迹走, 所有残差归零。
 *  2. 阈值从虚警率反解, 不从"米数"拍 (见 Thresholds 与 README)。
 *  3. 不知道自己在哪时必须说不知道 (DEGRADED ≠ TRUSTED)。
 */
class SpoofDetector(val config: DetectorConfig = DetectorConfig()) {

    enum class Phase { BOOTSTRAP, TRACKING, LOST }

    private var envCls = EnvClassifier()
    private var nav: NavEstimator? = null
    private var lastFrame: SensorFrame? = null
    private var env: EnvClass = EnvClass.OPEN_SKY
    private var phase = Phase.BOOTSTRAP

    private var cusumSpeedWheel: Cusum? = null
    private var cusumPos: Cusum? = null
    private var cusumVel: Cusum? = null
    private var cusumCourse: Cusum? = null
    private var cusumRange: Cusum? = null
    private var cusumMap: Cusum? = null
    private var cusumAlt: Cusum? = null
    private var cusumRf: Cusum? = null
    private var cusumRx: Cusum? = null
    private var cusumMotion: Cusum? = null
    private var cusumTime: Cusum? = null
    private var lastCusumEnv: EnvClass? = null

    private var lastChiPos = 0.0
    private var lastChiVel = 0.0

    private val fusion = EvidenceFusion(config.fusionDecayHalfLifeS)

    private var verdict = Verdict.DEGRADED
    private var suspectSince = Double.NaN
    private var confirmSince = Double.NaN

    private var hasAnchor = false
    private var outageStart = Double.NaN
    private var outageCount = 0
    private var longestOutage = 0.0
    private var anchorCount = 0

    private var prevPrn: IntArray = IntArray(0)
    private var bootstrapCleanFrames = 0

    // 气压高度与绝对高度的"慢变偏置"估计 (QNH 未知 / 天气漂移)
    private var altBias = 0.0
    private var altBiasVar = 30.0 * 30.0
    private var altRefSet = false

    private var corridor: List<DoubleArray>? = null
    private var corridorHalfWidth = Double.NaN
    private var roads: io.gnsstrust.sim.RoadNetwork? = null

    /** 锚点台账: 真实工程里来自运营商与自建基站清单 */
    var anchorPositions: Map<Long, DoubleArray>? = null
    var anchorKinds: Map<Long, AnchorKind>? = null

    fun attachMap(r: io.gnsstrust.sim.RoadNetwork?) { roads = r }
    fun attachCorridor(pts: List<DoubleArray>?, halfWidthM: Double) {
        corridor = pts; corridorHalfWidth = halfWidthM
    }

    // ==================================================================
    //  主流程
    // ==================================================================

    fun update(f: SensorFrame): SpoofReport {
        val prev = lastFrame
        lastFrame = f

        env = envCls.classify(f)
        ensureNav(f)
        rebuildCusumsIfEnvChanged()
        fusion.tick(f.t)

        val navE = nav!!
        pushVelocity(f)

        // ---- GNSS 质量分级
        val q = gnssQuality(f)
        if (q == GnssQuality.NONE) {
            enterLost(f)
        } else if (phase == Phase.LOST) {
            if (outageStart.isNaN()) outageStart = f.t
            val dur = f.t - outageStart
            if (dur > longestOutage) longestOutage = dur
            outageCount++
            outageStart = Double.NaN
            if (!hasAnchor || dur > config.lostReacquireS) {
                // 从未锚定, 或失锁太久 (惯导已漂到不可信):
                // **不能**假装还在跟踪 —— 必须重新走冷启动流程。
                phase = Phase.BOOTSTRAP
                bootstrapCleanFrames = 0
                hasAnchor = false
                navE.forcedColdStart()
                navE.x[0] = f.gnss.eM; navE.x[1] = f.gnss.nM
            } else {
                // 短时失锁 (过隧道): 保留锚点, 回到跟踪; 位置类 σ 由 recoveryRamp 放大
                phase = Phase.TRACKING
            }
        }

        // ---- 传感器传播 (所有状态都做)
        stepSensors(navE, f)

        if (q == GnssQuality.NONE) {
            // 失锁: 不产生任何位置类证据
            clearPositionCusums()
            // ★ 必须走 state machine: 失锁期间判决应当是 DEGRADED, 而不是
            // 沿用失锁前的 TRUSTED。开发中实际出现过"整段隧道都报 TRUSTED",
            // 那是把"不知道"说成了"没问题"。
            updateVerdict(f.t, gnssUsable = false)
            return emit(f, gnssUsable = false, meas = emptyMap())
        }

        // ---- 分状态处理
        val meas = LinkedHashMap<DetectorId, Double>()
        // 质量差的解 (2D / 卫星少 / 自报精度很差) 只能用来做"弱"检验:
        // 射频指纹、接收机自检、运动学、地图。位置/速度/航向/测距一律不做,
        // 因为它们的误差模型根本不成立。
        if (q == GnssQuality.POOR) {
            rfCheck(f)?.let { (z, g2) -> if (z > g2) meas[DetectorId.RF_ANOMALY] = z / g2 }
            rxCheck(f)?.let { (z, g2) -> if (z > g2) meas[DetectorId.RX_SELFCHECK] = z / g2 }
            for ((id, v) in meas) fusion.contribute(id, v)
            updateVerdict(f.t, gnssUsable = true)
            return emit(f, gnssUsable = true, meas = meas)
        }
        when (phase) {
            Phase.BOOTSTRAP -> handleBootstrap(f, prev, meas)
            Phase.TRACKING, Phase.LOST -> handleTracking(f, prev, meas)
        }

        for ((id, v) in meas) fusion.contribute(id, v)
        updateVerdict(f.t, gnssUsable = true)

        return emit(f, gnssUsable = true, meas = meas)
    }

    // ==================================================================
    //  状态处理
    // ==================================================================

    /**
     * BOOTSTRAP: 冷启动, 还没有可信锚点。
     *
     * 这里能做的事比想象的少 —— 但也比"什么都不能做"多:
     *  - 射频指纹 (不需要位置)
     *  - 接收机自检 (残差、钟差)
     *  - 运动学 (轮速 vs 报告速度)
     *  - 地图 (报告位置是否在路上)
     *  - 独立测距**不能用** (没有惯导位置可作参照)
     *
     * 通过"干净启动"的条件才能建立第一个锚点。
     */
    private fun handleBootstrap(f: SensorFrame, prev: SensorFrame?, meas: MutableMap<DetectorId, Double>) {
        val navE = nav!!

        // ---- 与位置无关的检验
        rfCheck(f)?.let { (z, g) -> if (z > g) meas[DetectorId.RF_ANOMALY] = z / g }
        rxCheck(f)?.let { (z, g) -> if (z > g) meas[DetectorId.RX_SELFCHECK] = z / g }
        motionCheck(f, prev)?.let { (z, g) -> if (z > g) meas[DetectorId.MOTION_PLAUSIBILITY] = z / g }
        mapCheck(f)?.let { (z, g) -> if (z > g) meas[DetectorId.MAP_VS_INS] = z / g }

        // 只在**所有**不依赖位置的检验都干净时, 才累计 bootstrap 进度。
        // 任一检验报警都会清零重来。
        //
        // ★ 注意: bootstrap 期间同样会把 meas 送进 evidence fusion, 因此若
        // 连续异常, 融合分会在 state machine 里独立地升到 SUSPECT ——
        // 这正是为了防止"开机即被骗"导致永远 bootstrap 不完、永远不报警的
        // 死锁 (开发中实测过: 分数累积到 3400+, 判决却始终停在 BOOTSTRAP)。
        val dirty = meas.isNotEmpty()
        if (dirty || gnssQuality(f) == GnssQuality.POOR) {
            bootstrapCleanFrames = 0
        } else {
            bootstrapCleanFrames++
        }

        // 必须在开阔环境 (或至少不是地库) 且连续干净才允许建立锚点
        val envOk = env != EnvClass.INDOOR_GARAGE && !f.inGarage
        if (bootstrapCleanFrames >= config.bootstrapCleanFrames && envOk) {
            anchorNow(f, f.gnss.posSigmaM)
            phase = Phase.TRACKING
        }
    }

    /**
     * TRACKING: 有锚点, 全部检验生效。
     */
    private fun handleTracking(f: SensorFrame, prev: SensorFrame?, meas: MutableMap<DetectorId, Double>) {
        val navE = nav!!
        val g = f.gnss

        // 机动检测: 陀螺角速率或航向/航迹差超阈值时, 航向必然有滞后。
        // 此时**关闭一切依赖航向方向的检验**, 否则每个路口都会误报。
        val yawRateDeg = abs(Math.toDegrees(f.imu.gz))
        val courseGap = wrapPi(g.courseRad - navE.x[4])
        val maneuvering = yawRateDeg > 3.0 || abs(courseGap) > Math.toRadians(20.0)

        // ---------------- (A) 位置 vs 惯导
        val sigmaPos = positionSigma(navE)
        val de = g.eM - navE.x[0]
        val dn = g.nM - navE.x[1]
        val chi = (de * de + dn * dn) / (sigmaPos * sigmaPos)
        lastChiPos = chi
        val chiGate = Thresholds.SINGLE_POINT_CHI2 * config.chi2Safety()
        if (chi > chiGate) bump(meas, DetectorId.POS_VS_INS, chi / chiGate)

        // 归一化标量创新: 投影到惯导航向上 (抓纵向拉偏最敏感)
        val c = cos(navE.x[4]); val s = sin(navE.x[4])
        val eAlong = (de * c + dn * s) / sigmaPos
        val eCross = (-de * s + dn * c) / sigmaPos
        if (drUsable(navE)) {
            cusumPos?.let {
                val firedP = it.update(max(eAlong, eCross) * 0.5 + min(eAlong, eCross) * 0.5, g.t)
                if (firedP) bump(meas, DetectorId.POS_VS_INS, it.level())
                else if (it.level() > 0.55) bump(meas, DetectorId.POS_VS_INS, it.level())
            }
        }

        // ---------------- (B) 多普勒速度 vs 惯导/轮速
        velocityChecks(navE, f, meas)

        // ---------------- (C) 磁罗盘偏置补偿
        //
        // 车上的磁罗盘有硬铁偏置 (±8°), 在本次仿真里约 3.5°。14 m/s 下
        // 这是 0.85 m/s 的横向速度偏差, 一分钟 50 m —— 是最大的假残差来源。
        //
        // 补偿手段必须是**只用非 GNSS 信息**: 陀螺在转弯时给出精确的
        // 航向增量, 而磁罗盘在短时间内的*变化*不受硬铁偏置影响 (偏置是
        // 常值)。所以对 (磁航向 - 陀螺积分航向) 的**长期平均**就是偏置本身。
        // 这个方法完全不用 GNSS, 攻击者无法通过射频影响它。
        f.mag?.let { m ->
            if (!m.disturbed && navE.speed() > 3.0) navE.learnMagBias(compassToPsi(m.headingDeg))
        }

        // ---------------- (C2) 航向一致性
        //
        // ★ 无人机没有磁罗盘, 在没有其它绝对航向来源时, "GNSS course vs
        // 惯导航向" 的差异会同时来自**惯导航向本身不可观测**与**GNSS 被欺骗**,
        // 二者无法区分。此时这一路检验必须禁用 (否则必然误报)。
        // 真实系统里补上这一路的前提是: 磁罗盘/视觉里程计/C2 来向中的任意一个。
        val headingObservable = f.platform == Platform.TRUCK || config.droneHasHeadingSource
        if (headingObservable && navE.speed() > config.minSpeedForCourseMps && !maneuvering) {
            // 用磁航向的误差作为 GNSS 航向检验的 σ。磁罗盘在本场景里已经
            // 通过滤波器持续修正航向, 所以惯导航向的 σ 很小 —— 但在峡谷里
            // 磁受扰, 要把 σ 放大。这是"没有绝对航向参照时不能判航向"的体现。
            val magQuality = lastFrame?.mag?.let { if (it.disturbed) 12.0 else 4.0 } ?: 8.0
            val dpsi = wrapPi(g.courseRad - navE.x[4])
            val sPsi = max(Math.toRadians(magQuality), navE.headingSigmaRad())
            val z = abs(dpsi) / sPsi
            val gate = Thresholds.SINGLE_SCALAR_Z
            if (z > gate) bump(meas, DetectorId.COURSE_VS_HEADING, z / gate)
            cusumCourse?.let {
                if (it.update(z * (if (dpsi < 0) -1.0 else 1.0), f.t))
                    bump(meas, DetectorId.COURSE_VS_HEADING, 1.0 + it.level())
            }
        }

        // ---------------- (D) 独立测距 + 仲裁
        rangeCheck(navE, f)?.let { (strength, signedZ, insShouldReanchor) ->
            if (insShouldReanchor) {
                // 惯导漂得比测距误差还离谱 => 我的参照坏了 => 用测距+GNSS 重锚
                reanchorFromRange(navE, f)
            } else {
                if (strength > 1.0) bump(meas, DetectorId.RANGE_VS_INS, strength)
                cusumRange?.let {
                    if (it.update(signedZ, f.t)) bump(meas, DetectorId.RANGE_VS_INS, 1.0 + it.level())
                }
            }
        }

        // ---------------- (E) 地图
        mapCheck(f)?.let { (z, gate) -> if (z > gate) bump(meas, DetectorId.MAP_VS_INS, z / gate) }

        // ---------------- (F) 高度
        altCheck(f)?.let { (z, gate) ->
            if (z > gate) bump(meas, DetectorId.ALT_VS_BARO, z / gate)
            // ★ CUSUM 必须吃**归一化**残差 (以 σ 计)。原样传米数会让统计量
            // 的量纲与门槛 (按 σ 标定) 不匹配, 几米的正常高度差就会被
            // 当成"每个采样 6σ"而迅速累积到报警。
            cusumAlt?.let {
                if (it.update(z / max(gate, 1e-6), f.t)) bump(meas, DetectorId.ALT_VS_BARO, 1.0 + it.level())
            }
        }

        // ---------------- (G) 射频 / 接收机 / 运动学 / 时间
        rfCheck(f)?.let { (z, gate) ->
            if (z > gate) bump(meas, DetectorId.RF_ANOMALY, z / gate)
            cusumRf?.let {
                if (it.update(z, f.t)) bump(meas, DetectorId.RF_ANOMALY, 1.0 + it.level())
            }
        }
        rxCheck(f)?.let { (z, gate) ->
            if (z > gate) bump(meas, DetectorId.RX_SELFCHECK, z / gate)
            cusumRx?.let {
                if (it.update(z, f.t)) bump(meas, DetectorId.RX_SELFCHECK, 1.0 + it.level())
            }
        }
        motionCheck(f, prev)?.let { (z, gate) ->
            if (z > gate) bump(meas, DetectorId.MOTION_PLAUSIBILITY, z / gate)
            cusumMotion?.let {
                if (it.update(z, f.t)) bump(meas, DetectorId.MOTION_PLAUSIBILITY, 1.0 + it.level())
            }
        }
        timeCheck(f, prev)?.let { (z, gate) -> if (z > gate) bump(meas, DetectorId.TIME_SYNC, z / gate) }

        // ---------------- 重锚
        maybeReanchor(navE, f, meas)
    }

    /**
     * 重锚: 承认 "GNSS 现在是对的", 把惯导拉回来。
     *
     * 这是双刃剑:
     *  - 不做: 惯导漂到比 GNSS 还不准, 检测失效;
     *  - 乱做: 攻击者只要让轨迹自洽就能"带跑"检测器。
     *
     * 所以条件比报警更严: 硬证据零异常 + 融合分低 + 不在刚失锁恢复期。
     */
    private fun maybeReanchor(navE: NavEstimator, f: SensorFrame, meas: Map<DetectorId, Double>) {
        // 硬证据一响就绝不重锚 —— 否则等于把攻击者的假轨迹写进参照
        if (meas.keys.any {
                it == DetectorId.RANGE_VS_INS || it == DetectorId.RX_SELFCHECK ||
                    it == DetectorId.MAP_VS_INS || it == DetectorId.RF_ANOMALY
            }) return
        if (verdict >= Verdict.SUSPECT) return

        // ★★★ 重锚策略
        //
        // 两个极端都不对:
        //   - 每帧重锚 => 惯导变成"带 GNSS 噪声的 GNSS", 残差自相关, 检测器失去
        //     独立性 (会被攻击者的自洽假轨迹带跑);
        //   - 只在"独立参照支持 GNSS"时才重锚 => 惯导自由漂到 100 m 以上,
        //     残差被惯导自身的漂移淹没, 同样失效。
        //
        // 正确做法: **重锚本身不是判决, 它只是"把两个估计融合"**。
        // 真正决定可信度的是: 融合残差是否**持续**超出误差预算。
        // 所以:
        //   1. 只要融合残差低于卡方门槛, 就正常融合 (这是所有导航系统该做的事);
        //   2. 一旦残差超门槛, 立刻停止重锚, 并开始累积"不可解释的偏移";
        //   3. 这个累积量才是报警的依据。
        //
        // 这样检测器对攻击者的响应是: 从"残差出现的那一帧"起, 惯导就不再被
        // 带偏, 偏移量随后随时间线性增长, 很快越过门槛 —— 见 README 的推导。
        val ar = rangeArbiter(navE, f)
        val residualZ = positionChi2(navE, f)   // 马氏距离 (2 自由度)
        val fusionOk = residualZ < Thresholds.SINGLE_POINT_CHI2
        val rangeSaysOk = !ar.hasRange || ar.gnssZ <= config.rangeGate * 1.5
        val rangeSaysBad = ar.hasRange && ar.gnssZ > config.rangeGate * 2.0

        if (rangeSaysBad) {
            // 独立参照明确反对 => 绝不重锚
            lastAnchorReason = "reject(测距反对: gZ=%.1f)".format(ar.gnssZ)
            return
        }
        if (!fusionOk) {
            // 残差超门槛 => 停止融合, 保留惯导的独立性
            lastAnchorReason = "reject(残差 chi2=%.1f)".format(residualZ)
            if (navE.timeSinceAnchor > config.maxDrForGateS * 1.5) restartBootstrap()
            return
        }
        if (!rangeSaysOk) {
            lastAnchorReason = "reject(测距存疑)"
            return
        }
        lastAnchorReason = "accept(chi2=%.1f)".format(residualZ)
        val anchorSigma = sqrt(f.gnss.posSigmaM.pow(2) + navE.posSigmaM().pow(2) * 0.35)
            .coerceAtLeast(2.5)
        anchorNow(f, anchorSigma)
        // 重锚把 GNSS 噪声注入位置: 把协方差抬回锚定噪声量级,
        // 诚实反映"我的位置里含有 GNSS 的噪声"。
        val inflate = anchorSigma * anchorSigma
        navE.P[0, 0] = max(navE.P[0, 0], inflate)
        navE.P[1, 1] = max(navE.P[1, 1], inflate)
    }

    /** 惯导已不可信时, 直接以 GNSS + 测距支持的融合结果重锚 */
    private fun reanchorFromRange(navE: NavEstimator, f: SensorFrame) {
        if (!f.gnss.valid) return
        anchorNow(f, sqrt(f.gnss.posSigmaM.pow(2) + 25.0))
        val inflate = f.gnss.posSigmaM.pow(2) + 25.0
        navE.P[0, 0] = max(navE.P[0, 0], inflate)
        navE.P[1, 1] = max(navE.P[1, 1], inflate)
        cusumPos?.reset(); cusumVel?.reset(); cusumCourse?.reset()
    }

    /** 重锚/初始化: 承认某次 GNSS 解, 把惯导拉过去 */
    private fun anchorNow(f: SensorFrame, sigma: Double) {
        val navE = nav!!
        navE.anchorTo(f.gnss, sigma)
        if (f.gnss.valid && f.gnss.fixType != FixType.TWO_D) {
            navE.vert?.anchorTo(f.gnss.altM, max(4.0, f.gnss.vertSigmaM))
            altRefSet = true
            altBias = 0.0
            altBiasVar = 4.0 * 4.0
        }
        hasAnchor = true
        anchorCount++
    }

    /** 重新走冷启动 (惯导已不可用) */
    private fun restartBootstrap() {
        phase = Phase.BOOTSTRAP
        bootstrapCleanFrames = 0
        hasAnchor = false
        nav!!.forcedColdStart()
    }

    private data class RangeArbiter(
        val gnssZ: Double,
        val insZ: Double,
        /** 带符号的 GNSS-测距归一化残差 (给 CUSUM 累积用) */
        val gnssSigned: Double,
        val hasRange: Boolean,
    )

    /**
     * 惯导位置的**诚实**不确定度。
     *
     * 本滤波器持续接收轮速与磁罗盘, 所以 P 里的位置方差增长得慢 (几米);
     * 但实际漂移主要来自**未建模的系统项**: 磁罗盘残余硬铁偏置、轮胎滚动
     * 半径随载重/胎压缓变导致的轮速标度误差、陀螺零偏残余。这些在 P 里
     * 没有对应过程噪声, 所以 P 会严重低估漂移。
     *
     * 诚实做法: 显式加上"速度标度漂移"与"航向漂移"两项解析增长率。
     * 这两个系数是**在仿真中回放标定**出来的 (3.5% 量级的等效距离误差),
     * 真实工程里应当用实车采集的数据重新标定 —— 它们直接决定"多久必须重锚"。
     */
    private fun insPosSigma(navE: NavEstimator): Double {
        val t = navE.timeSinceAnchor.coerceAtMost(600.0)
        val v = max(navE.speed(), 1.0)
        val epsV = 0.02
        val epsPsi = Math.toRadians(2.0)
        val driftRate = v * sqrt(epsV * epsV + epsPsi * epsPsi) + 0.15
        val analytic = driftRate * t + 2.0
        return sqrt(navE.posSigmaM().pow(2) + analytic.pow(2))
    }

    /** 位置创新的总 σ: 惯导 + GNSS 自报 + 环境相关多径预算 + 失锁恢复惩罚 */
    private fun positionSigma(navE: NavEstimator): Double {
        val pDr = insPosSigma(navE)
        val g = lastFrame!!.gnss
        val rGnss = g.posSigmaM
        val bEnv = config.envCorrelatedSigma(env)
        val sinceOutage = if (outageStart.isNaN()) Double.MAX_VALUE else lastFrame!!.t - outageStart
        val recov = if (sinceOutage < config.recoveryRampS)
            20.0 * (1.0 - sinceOutage / config.recoveryRampS) else 0.0
        return sqrt(pDr * pDr + rGnss * rGnss + bEnv * bEnv + recov * recov)
    }

    /** 位置创新的马氏距离平方 (2 自由度卡方统计量) */
    private fun positionChi2(navE: NavEstimator, f: SensorFrame): Double {
        val de = f.gnss.eM - navE.x[0]
        val dn = f.gnss.nM - navE.x[1]
        val s = positionSigma(navE)
        return (de * de + dn * dn) / (s * s)
    }

    /** 惯导是否还适合作为比较基准 */
    private fun drUsable(navE: NavEstimator): Boolean =
        navE.timeSinceAnchor < config.maxDrForGateS && navE.posSigmaM() < config.maxDrSigmaForGate

    private fun rangeArbiter(navE: NavEstimator, f: SensorFrame): RangeArbiter {
        val anchors = anchorPositions ?: return RangeArbiter(0.0, 0.0, 0.0, false)
        if (f.ranges.isEmpty()) return RangeArbiter(0.0, 0.0, 0.0, false)

        var bestGnss = 0.0
        var bestIns = 0.0
        var signedGnss = 0.0
        for (r in f.ranges) {
            val a = anchors[r.anchorId] ?: continue
            val dGnss = hypot(f.gnss.eM - a[0], f.gnss.nM - a[1])
            val dIns = hypot(navE.x[0] - a[0], navE.x[1] - a[1])
            // 误差预算: 测距自身 + 各位置解的不确定度
            val sG = sqrt(r.sigmaM.pow(2) + f.gnss.posSigmaM.pow(2) * 0.6 + (39.0/3.46).pow(2))
            val sI = sqrt(r.sigmaM.pow(2) + insPosSigma(navE).pow(2))
            val resG = dGnss - r.rangeM
            val resI = dIns - r.rangeM
            val zg = abs(resG) / sG
            val zi = abs(resI) / sI
            if (zg > bestGnss) { bestGnss = zg; signedGnss = resG / sG }
            if (zi > bestIns) bestIns = zi
        }
        return RangeArbiter(bestGnss, bestIns, signedGnss, true)
    }

    /**
     * 独立测距的判据 —— 慢速拉偏的主要抓手。
     *
     * 实现要点 (每一条都是踩坑换来的):
     *  1. **只在惯导自由运行时才算**。若每帧都重锚, 惯导位置就等于"带 GNSS
     *     噪声的 GNSS 位置", 残差退化成自相关序列, CUSUM 会在干净数据上
     *     单调累积到报警 —— 这是本模块最隐蔽的一个假警源。
     *  2. TA 量化步长 39 m > 有效噪声, 逐帧比较几乎全是量化跳变, 所以
     *     必须让滤波器**累积**多帧信息, CUSUM 正是干这个的, 但要按量化
     *     后的有效 σ 归一化。
     *  3. 返回带符号残差 (以 σ 计), 供 CUSUM 使用。
     */
    private fun rangeCheck(navE: NavEstimator, f: SensorFrame): Triple<Double, Double, Boolean>? {
        val ar = rangeArbiter(navE, f)
        if (!ar.hasRange) return null
        val insIsWorse = ar.insZ > config.rangeGate && ar.insZ > ar.gnssZ * 1.3
        if (insIsWorse) return Triple(0.0, 0.0, true)
        // ★ 只在惯导自由运行 (足够久没被 GNSS 重锚) 时, 残差才是"独立证据"
        val freeRunning = navE.timeSinceAnchor > config.rangeFreeRunS
        val strength = if (freeRunning) ar.gnssZ / config.rangeGate else 0.0
        val cusumInput = if (freeRunning) ar.gnssSigned else 0.0
        return Triple(strength, cusumInput, false)
    }

    /**
     * 地图约束: 车辆应当落在路网上; 无人机应当在任务走廊内。
     * 地图来自本地地图库, 射频欺骗改不了它。
     */
    private fun mapCheck(f: SensorFrame): Pair<Double, Double>? {
        val g = f.gnss
        // 地库/停车场里地图不可靠
        if (f.inGarage) return null
        if (f.platform == Platform.TRUCK) {
            val rn = roads ?: return null
            val (dist, halfW) = rn.nearest(g.eM, g.nM)
            val allowed = halfW + 10.0 + 2.5 * g.posSigmaM
            return dist to allowed
        } else {
            val cor = corridor ?: return null
            val half = corridorHalfWidth
            if (half.isNaN()) return null
            val pr = Polyline.project(cor, doubleArrayOf(g.eM, g.nM))
            val allowed = half + 2.5 * g.posSigmaM
            return pr.dist to allowed
        }
    }

    /**
     * 高度检验。关键是**必须先扣掉未知的绝对偏置** (QNH/气压随天气漂),
     * 所以这里检验的是"高度差的变化"而不是"高度差"本身。
     */
    /**
     * 高度检验。
     *
     * 两个陷阱:
     *  1. 气压高度有**未知的绝对偏置** (QNH + 天气漂移, 几十分钟可达十几米),
     *     所以绝对不能比较绝对高度。正确做法是比较**窗口内的高度变化**。
     *  2. 无人机的垂直通道会被水平机动干扰 (俯仰改变重力投影), 所以
     *     σ 必须包括机动项。
     *
     * "把无人机一点点引偏"经常先动高度, 因为高度上的异常不影响水平轨迹的
     * 视觉合理性 —— 所以这一路值得保留, 但必须做对。
     */
    private var altHistT = ArrayDeque<Double>()
    private var altHistGnss = ArrayDeque<Double>()
    private var altHistBaro = ArrayDeque<Double>()

    private fun altCheck(f: SensorFrame): Pair<Double, Double>? {
        val v = nav!!.vert ?: return null
        if (f.gnss.fixType == FixType.TWO_D) return null

        // ★ 这里必须做**两件事**, 缺一不可:
        //  (1) 绝对差检验: |gnssAlt - 本地气压高度| ——直接抓"高度被拉偏"。
        //      气压计有未知的绝对偏置 (QNH), 所以要先在线估计这个偏置。
        //  (2) 变化量检验: 窗口内两者的变化量之差 ——抓"偏置估计被追着跑"的情况。
        //
        // 只做 (2) 是**错的**: 若攻击者以恒定速率缓慢抬高高度, 窗口两端的
        // 变化量几乎相等, 变化量之差接近 0, 永远检不出来。开发中实测:
        // 高度被拉偏 100 m, 得分却始终为 0。必须靠 (1) 配合一个慢速的
        // 偏置估计器 —— 偏置更新要足够慢, 使真实的缓慢拉偏不被吸收。
        val rawDiff = f.gnss.altM - v.alt
        if (!altRefSet) {
            altBias = rawDiff
            altBiasVar = 20.0 * 20.0
            altRefSet = true
            return null
        }
        val innovation = rawDiff - altBias
        val sigmaAbs = sqrt(
            v.sigma().pow(2) + f.gnss.vertSigmaM.pow(2) +
            altBiasVar + config.envCorrelatedSigma(env).pow(2) * 3.0 + 4.0
        )
        val zAbs = abs(innovation) / sigmaAbs
        // 慢速更新偏置: z 大时几乎不动 (不让攻击者靠"等"把偏置拉过去)
        val k = if (zAbs > 2.5) 0.0001 else 0.002
        altBias += k * innovation
        altBiasVar = altBiasVar * (1 - k) + k * k * sigmaAbs * sigmaAbs
        altBiasVar = altBiasVar.coerceIn(1.0, 60.0 * 60.0)

        return zAbs to 3.5
    }

    /**
     * 射频指纹。完全不看轨迹, 只看信号本身。
     *
     *  - 星座换血 (PRN 集合 Jaccard 距离)
     *  - C/N0 分布过于整齐
     *  - 接收机"干净得反常" (残差低 + 卫星数贴最小值 + 不在开阔地)
     */
    private fun rfCheck(f: SensorFrame): Pair<Double, Double>? {
        if (env == EnvClass.TUNNEL || env == EnvClass.INDOOR_GARAGE) return null
        val g = f.gnss
        if (!g.valid || g.prn.isEmpty()) { prevPrn = g.prn.copyOf(); return null }
        var score = 0.0

        if (prevPrn.isNotEmpty() && prevPrn.size >= 4) {
            val a = prevPrn.toSet(); val b = g.prn.toSet()
            val inter = a.intersect(b).size.toDouble()
            val union = a.union(b).size.toDouble().coerceAtLeast(1.0)
            val jaccard = inter / union
            if (jaccard < 0.34) score += (0.34 - jaccard) * 9.0
        }
        prevPrn = g.prn.copyOf()

        if (g.cnoDbHz.size >= 5) {
            val m = g.cnoDbHz.average()
            val sd = sqrt(g.cnoDbHz.sumOf { (it - m) * (it - m) } / g.cnoDbHz.size)
            if (sd < 1.1 && m > 39.0) score += (1.1 - sd) * 2.5
        }
        if (g.rejectedSats == 0 && g.sats in 4..5 && g.residualRmsM < 1.4 &&
            (env == EnvClass.URBAN_CANYON || env == EnvClass.SUBURBAN)) {
            score += 1.0
        }
        return if (score <= 0.0) null else score to config.rfGate
    }

    private fun rxCheck(f: SensorFrame): Pair<Double, Double>? {
        val g = f.gnss
        var z = 0.0
        val base = when (env) {
            EnvClass.OPEN_SKY -> 1.3; EnvClass.SUBURBAN -> 2.0
            EnvClass.URBAN_CANYON -> 3.8; else -> 6.0
        }
        z = max(z, (g.residualRmsM / base - 2.2).coerceAtLeast(0.0) * 1.5)
        if (g.clockJumpM > 5.0) z += (g.clockJumpM / 5.0 - 1.0) * 2.5
        return if (z <= 0.0) null else z to 1.0
    }

    /**
     * 运动学不可能:
     *  - 报告速度超平台上限
     *  - **窗口位移**与积分多普勒速度不符 (位置跳变但速度没跟)
     *  - 轮速说停住, GNSS 说在跑
     *
     * 关键实现细节: 绝不能用**相邻两帧**做位置差分。GPS 位置的白噪声
     * 在 10 Hz 下会让 |Δpos|/Δt 出现 40 m/s 的抖动, 这是最常见的
     * 假警来源 (本次开发中就踩了这个坑)。正确做法是取 T≈2 s 的窗口:
     * 白噪声被平均掉 (σ_位移 ≈ σ_pos·√2, 与窗口起点无关), 而真实
     * 速度差在 2 s 内会累积成几十米的可观位移。
     */
    private fun motionCheck(f: SensorFrame, prev: SensorFrame?): Pair<Double, Double>? {
        val g = f.gnss
        var s = 0.0
        val vmax = if (f.platform == Platform.DRONE) 27.0 else 33.0
        if (g.speedMps > vmax) s += (g.speedMps / vmax - 1.0) * 4.0

        val w = f.wheel
        if (w != null && !w.slip && w.speedMps < 0.05 && g.speedMps > 1.5) {
            s += (g.speedMps / 1.5) * 2.0
        }
        return if (s <= 0.0) null else s to 1.0
    }

    /**
     * 时间一致性: 窗口位移 vs 积分多普勒速度。
     *
     * 定义 (窗口 W = 2 s, 每 10 帧推进一次):
     *   dPos   = x(t) - x(t-W)          真实的位置增量
     *   dVel   = ∫ v_doppler dt         Doppler 速度积分出的位移
     *   r      = |dPos - dVel|
     *
     * 只偏位置不偏 Doppler 的攻击者, 其注入的偏移量会几乎全部出现在 r 里。
     * 若他连 Doppler 一起伪造 (一致欺骗), 这一路失效 —— 这是已知且刻意的
     * 局限, 因为任何纯射频层的自洽性都无法识破一致欺骗。
     *
     * σ_r 由位置噪声与速度噪声共同决定:
     *   σ_r² ≈ 2σ_pos² + σ_v²·W² + (drift)²
     * 其中 2σ_pos² 来自窗口两端的独立噪声; 相关多径项按环境加入。
     */
    private val posHist = ArrayDeque<Pair<Double, DoubleArray>>()

    private fun timeCheck(f: SensorFrame, prev: SensorFrame?): Pair<Double, Double>? {
        val g = f.gnss
        if (!g.valid) return null
        posHist.addLast(f.t to doubleArrayOf(g.eM, g.nM))
        // 只保留窗口内的点
        while (posHist.isNotEmpty() && posHist.first().first < f.t - config.velocityWindowS - 1.0) {
            posHist.removeFirst()
        }
        if (f.t < config.velocityWindowS + 1.0) return null
        // 找最接近 t-W 的样本
        val target = f.t - config.velocityWindowS
        val old = posHist.minByOrNull { abs(it.first - target) } ?: return null
        if (abs(old.first - target) > 0.35) return null
        val dtW = f.t - old.first
        if (dtW < 1.0) return null

        // 用多普勒速度积分: 近似用两端速度的平均 (速度变化在 2 s 内很小)
        val p = prev?.gnss
        val vE = g.speedMps * cos(g.courseRad)
        val vN = g.speedMps * sin(g.courseRad)
        val vE0 = if (p != null && p.valid) p.speedMps * cos(p.courseRad) else vE
        val vN0 = if (p != null && p.valid) p.speedMps * sin(p.courseRad) else vN
        // 窗口两端速度的平均
        val vEm = if (velBufE.isEmpty()) vE else winVelE
        val vNm = if (velBufN.isEmpty()) vN else winVelN
        val dVelE = vEm * dtW
        val dVelN = vNm * dtW
        val dPosE = g.eM - old.second[0]
        val dPosN = g.nM - old.second[1]
        val r = hypot(dPosE - dVelE, dPosN - dVelN)

        val sigPos = sqrt(2.0) * g.posSigmaM
        val sigVel = g.speedSigmaMps * dtW
        val sigMp = config.envCorrelatedSigma(env) * 0.6
        val sigma = sqrt(sigPos * sigPos + sigVel * sigVel + sigMp * sigMp + 1.0)
        return r to (sigma * 4.0)
    }

    /**
     * 滑动窗口内的平均多普勒速度。
     *
     * 用固定长度的环形缓冲, 长度受 velocityWindowS 与采样率约束 ——
     * 不随运行时长增长 (早期版本用一个以时间戳为 key 的 HashMap,
     * 每帧插入一条永不清理, 长时间运行会持续吃内存)。
     */
    private val velBufT = ArrayDeque<Double>()
    private val velBufE = ArrayDeque<Double>()
    private val velBufN = ArrayDeque<Double>()
    private var winVelE = 0.0
    private var winVelN = 0.0

    private fun pushVelocity(f: SensorFrame) {
        val g = f.gnss
        if (!g.valid) { winVelE = 0.0; winVelN = 0.0; return }
        velBufT.addLast(f.t)
        velBufE.addLast(g.speedMps * cos(g.courseRad))
        velBufN.addLast(g.speedMps * sin(g.courseRad))
        while (velBufT.isNotEmpty() && velBufT.first() < f.t - config.velocityWindowS) {
            velBufT.removeFirst(); velBufE.removeFirst(); velBufN.removeFirst()
        }
        winVelE = velBufE.average()
        winVelN = velBufN.average()
    }

    private fun stepSensors(navE: NavEstimator, f: SensorFrame) {
        navE.propagate(f)
        if (f.platform == Platform.TRUCK) {
            navE.updateNonHolonomic(f.platform, navE.speed())
            f.wheel?.let { w ->
                navE.updateWheel(w)
                if (w.speedMps < 0.08 && !w.slip && navE.speed() < 0.7) navE.updateZupt()
            }
        } else {
            f.flowVelMps?.let { flow ->
                navE.updateOpticalFlow(flow, f.flowSigmaMps, f.baro.altM)
                // 悬停时的零速修正: 光流说"几乎没动"时, 把水平速度往 0 拉。
                // 否则光流的白噪声会被积分成 1-2 m/s 的假速度, 在长时间的
                // 悬停里越积越多, 最后与 GNSS 的 0 速度对不上而误报。
                if (hypot(flow[0], flow[1]) < max(0.6, f.flowSigmaMps * 2.0))
                    navE.updateHorizontalZUPT()
            }
        }
        f.mag?.let { m ->
            val corrected = navE.correctedMagPsi(compassToPsi(m.headingDeg))
            navE.updateHeading(MagObs(m.t, psiToCompass(corrected), m.sigmaDeg, m.disturbed))
        }
        navE.vert?.updateBaro(f.baro, 4.0)
    }

    private fun isManeuvering(f: SensorFrame, navE: NavEstimator): Boolean {
        val yawRateDeg = abs(Math.toDegrees(f.imu.gz))
        val gap = abs(wrapPi(f.gnss.courseRad - navE.x[4]))
        return yawRateDeg > 3.0 || gap > Math.toRadians(20.0)
    }

    private fun velocityChecks(navE: NavEstimator, f: SensorFrame, meas: MutableMap<DetectorId, Double>) {
        val g = f.gnss
        val speed = navE.speed()
        if (speed > 0.8 || g.speedMps > 0.8) {
            val tsa = navE.timeSinceAnchor.coerceAtMost(120.0)
            val gnssDop = max(g.speedSigmaMps, 0.08)
            val wheelErr = if (f.wheel != null) 0.02 * navE.speed() + 0.05 else 0.0
            val freeRunDrift = 0.03 * navE.speed() * sqrt(tsa / 60.0) +
                (if (f.platform == Platform.DRONE && f.flowVelMps == null)
                    0.8 * sqrt(navE.timeSinceVelocityObs.coerceAtMost(60.0)) else 0.0)
            val sv = sqrt(
                (1.6 * gnssDop).pow(2) + config.velSigma(env, tsa).pow(2) +
                    wheelErr.pow(2) + freeRunDrift.pow(2) + 0.12 * 0.12
            )
                val dSpeed = g.speedMps - speed
            // 低速时 (悬停/怠速) 速度模长本身很小, 相对误差被放大; 同时
            // GNSS 在低速下的 Doppler 也不可靠 (course 甚至没有定义)。
            // 因此低速段整体放宽 —— 这不是放水, 而是诚实: 在 0.5 m/s 的
            // 水平上谈论 0.3 m/s 的偏差没有意义。
            val lowSpeedRelax = if (min(g.speedMps, speed) < 2.0) 2.5 else 1.0
            val chiV = (dSpeed * dSpeed) / (sv * sv * lowSpeedRelax * lowSpeedRelax)
            lastChiVel = chiV
            val gateV = Thresholds.SINGLE_SCALAR_Z * Thresholds.SINGLE_SCALAR_Z
            if (chiV > gateV) bump(meas, DetectorId.VEL_VS_INS, chiV / gateV)
            // 悬停/静止时速度模长接近 0, 相对误差被无限放大, 没有可检验的信号。
            // 另外刚重锚的 2 帧内 (滤波器尚未被新观测拉平) 也跳过序列检验,
            // 否则会捕获重锚瞬间的暂态。只保留上面单点检验 (已有低速放宽)。
            if (drUsable(navE) && navE.timeSinceAnchor > 0.3 &&
                min(g.speedMps, speed) > 1.5) {
                cusumVel?.let {
                    if (it.update(dSpeed / sv, f.t)) bump(meas, DetectorId.VEL_VS_INS, 1.0 + it.level())
                }
            }
        }
        f.wheel?.let { w ->
            if ((w.speedMps > 0.35 || g.speedMps > 0.35) && !w.slip) {
                val dv = g.speedMps - w.speedMps
                val s = hypot(max(w.quantSigmaMps, 0.05) * 1.5, g.speedSigmaMps) + 0.08
                val z = abs(dv) / s
                val gate = Thresholds.SINGLE_SCALAR_Z
                if (z > gate) bump(meas, DetectorId.SPEED_VS_WHEEL, z / gate)
                cusumSpeedWheel?.let {
                    if (it.update(dv / s, f.t)) bump(meas, DetectorId.SPEED_VS_WHEEL, 1.0 + it.level())
                }
            }
        }
    }

    // ==================================================================
    //  判决与输出
    // ==================================================================

    private fun updateVerdict(t: Double, gnssUsable: Boolean) {
        val total = fusion.total()
        val th = config.thresholds
        val raw = when {
            !gnssUsable -> Verdict.DEGRADED
            // bootstrap 期间无法用惯导交叉验证位置, 因此正常情况下只能说"不知道"
            // (DEGRADED)。但如果连续多帧都被**不需要位置**的检验判为异常
            // (射频指纹/接收机自检/运动学/地图), 那必须报 SUSPECT —— 否则
            // "开机即被骗"就永远发现不了。
            total >= th.confirmed -> Verdict.CONFIRMED
            total >= th.suspect -> Verdict.SUSPECT
            phase == Phase.BOOTSTRAP && !hasAnchor -> Verdict.DEGRADED
            else -> Verdict.TRUSTED
        }
        if (raw > verdict) {
            if (raw == Verdict.CONFIRMED) {
                if (confirmSince.isNaN()) confirmSince = t
                if (t - confirmSince >= th.confirmHoldS) verdict = Verdict.CONFIRMED
                else if (verdict < Verdict.SUSPECT) verdict = Verdict.SUSPECT
            } else {
                if (suspectSince.isNaN()) suspectSince = t
                if (t - suspectSince >= th.suspectHoldS) verdict = Verdict.SUSPECT
            }
        } else if (raw < verdict) {
            if (verdict == Verdict.CONFIRMED) {
                // 确认后要连续 60 s 低分才降级
                if (confirmSince.isNaN()) confirmSince = t
                if (t - confirmSince > 60.0) { verdict = raw; confirmSince = Double.NaN }
            } else if (raw == Verdict.DEGRADED || total < th.suspect * 0.5) {
                verdict = raw
                suspectSince = Double.NaN
            }
        }
        if (verdict == Verdict.CONFIRMED) confirmSince = if (confirmSince.isNaN()) t else confirmSince
    }

    private fun emit(f: SensorFrame, gnssUsable: Boolean, meas: Map<DetectorId, Double>): SpoofReport {
        val navE = nav!!
        val total = fusion.total()
        val reasons = ArrayList<String>()
        if (!gnssUsable) reasons.add("GNSS 无有效定位, 仅惯导航位推算")
        if (phase == Phase.BOOTSTRAP) reasons.add("冷启动: 尚未建立可信锚点")
        for ((id, v) in meas.entries.sortedByDescending { it.value }.take(3)) {
            if (v > 0.6) reasons.add("${id.title} (%.1f×门槛)".format(v))
        }
        val unc = if (gnssUsable && phase == Phase.TRACKING && drUsable(navE))
            sqrt(insPosSigma(navE).pow(2) + config.envCorrelatedSigma(env).pow(2)) else Double.NaN
        return SpoofReport(
            t = f.t, verdict = verdict, confidence = confidenceOf(total, gnssUsable),
            score = total, dominant = fusion.dominant()?.first, reasons = reasons,
            navPosSigmaM = insPosSigma(navE), posUncertaintyM = unc,
            timeSinceAnchorS = if (navE.timeSinceAnchor.isFinite()) navE.timeSinceAnchor else Double.NaN,
            gnssUsable = gnssUsable, env = env,
            measurements = meas,
            cusum = mapOf(
                DetectorId.POS_VS_INS to (cusumPos?.level() ?: 0.0),
                DetectorId.VEL_VS_INS to (cusumVel?.level() ?: 0.0),
                DetectorId.RANGE_VS_INS to (cusumRange?.level() ?: 0.0),
                DetectorId.COURSE_VS_HEADING to (cusumCourse?.level() ?: 0.0),
            ),
            chi2Pos = lastChiPos, chi2Vel = lastChiVel,
            phase = phase.name,
        )
    }

    private fun confidenceOf(score: Double, usable: Boolean): Double {
        val th = config.thresholds
        if (!usable) return 0.85 // "不知道" 这件事本身是很有把握的
        return when {
            score >= th.confirmed * 1.5 -> 0.99
            score >= th.confirmed -> 0.9 + 0.09 * ((score - th.confirmed) / (th.confirmed * 0.5)).coerceIn(0.0, 1.0)
            score >= th.suspect -> 0.5 + 0.4 * ((score - th.suspect) / (th.confirmed - th.suspect)).coerceIn(0.0, 1.0)
            else -> (score / th.suspect * 0.5).coerceIn(0.0, 0.5)
        }
    }

    // ==================================================================
    //  基础设施
    // ==================================================================

    private fun ensureNav(f: SensorFrame) {
        if (nav != null) return
        // 注意: 这里**不**用第一个 GNSS 位置作为先验。
        // 冷启动时从地库/窄巷出来, 第一个定位解可能就是错的;
        // 把它当先验等于让攻击者只需要控制第一帧。
        // 位置先验给 0 且协方差极大, 等 bootstrap 通过后再重锚。
        nav = NavEstimator(f.platform, doubleArrayOf(0.0, 0.0), pickInitialHeading(f),
            NavConfig.forPlatform(f.platform))
        nav!!.forcedColdStart()
        f.baro.let { b -> nav!!.vert?.anchorTo(b.altM, 30.0) }
        buildCusums()
        phase = if (f.gnss.valid) Phase.BOOTSTRAP else Phase.LOST
        if (!f.gnss.valid) outageStart = f.t
    }

    private fun buildCusums() {
        val base = Thresholds.cusumFor(env, config.dt, config.arl0Hours, config.deltaStar, config.effTau(env))
        val step = base.effTau
        fun mk(id: DetectorId, kk: Double = base.kRef, hh: Double = base.h) = Cusum(id, kk, hh, step)
        cusumPos = mk(DetectorId.POS_VS_INS)
        cusumVel = mk(DetectorId.VEL_VS_INS)
        cusumCourse = mk(DetectorId.COURSE_VS_HEADING, base.kRef * 0.85, base.h)
        cusumRange = mk(DetectorId.RANGE_VS_INS, base.kRef, base.h * 0.75)
        cusumMap = mk(DetectorId.MAP_VS_INS)
        cusumAlt = mk(DetectorId.ALT_VS_BARO)
        cusumRf = mk(DetectorId.RF_ANOMALY, base.kRef * 0.7, base.h)
        cusumRx = mk(DetectorId.RX_SELFCHECK, base.kRef * 0.7, base.h)
        cusumSpeedWheel = mk(DetectorId.SPEED_VS_WHEEL)
        cusumMotion = mk(DetectorId.MOTION_PLAUSIBILITY)
        cusumTime = mk(DetectorId.TIME_SYNC, base.kRef, base.h * 0.8)
        lastCusumEnv = env
    }

    private fun rebuildCusumsIfEnvChanged() {
        if (lastCusumEnv != env) buildCusums()
    }

    private fun clearPositionCusums() {
        cusumPos?.reset(); cusumVel?.reset(); cusumRange?.reset()
        cusumCourse?.reset(); cusumRf?.reset()
    }

    private fun enterLost(f: SensorFrame) {
        if (phase != Phase.LOST) {
            if (outageStart.isNaN()) outageStart = f.t
            phase = Phase.LOST
            clearPositionCusums()
        }
    }

    private fun bump(m: MutableMap<DetectorId, Double>, id: DetectorId, v: Double) {
        val cur = m[id] ?: 0.0
        if (v > cur) m[id] = v
    }

    private fun gnssQuality(f: SensorFrame): GnssQuality {
        val g = f.gnss
        if (!g.valid || g.fixType == FixType.NONE) return GnssQuality.NONE
        if (g.fixType == FixType.TWO_D) return GnssQuality.POOR
        if (g.sats < 5) return GnssQuality.POOR
        if (g.posSigmaM > 25.0) return GnssQuality.POOR
        if (g.posSigmaM <= 8.0 && g.sats >= 7) return GnssQuality.GOOD
        return GnssQuality.OK
    }

    private fun pickInitialHeading(f: SensorFrame): Double =
        if (f.gnss.valid && f.gnss.speedMps > 1.0) f.gnss.courseRad
        else f.mag?.let { compassToPsi(it.headingDeg) } ?: 0.0

    /** 供外部/测试查询 */
    /** 最近一次重锚决策的原因 (诊断用) */
    var lastAnchorReason: String = ""

    fun say(): Phase = phase
    /** 融合账本 (诊断用) */
    fun fusionBreakdown(): Map<DetectorId, Double> = fusion.breakdown()
    fun navState(): DoubleArray? = nav?.let {
        doubleArrayOf(it.x[0], it.x[1], it.x[2], it.x[3], it.x[4], it.posSigmaM(),
            sqrt(max(0.0, it.P[4, 4])), sqrt(max(0.0, it.P[2, 2])), it.x[5], it.x[6], it.x[7])
    }
    fun stats(): Stats = Stats(anchorCount, outageCount, longestOutage)
    data class Stats(val anchors: Int, val outages: Int, val longestOutageS: Double)

    enum class GnssQuality { NONE, POOR, OK, GOOD }
}

package io.gnsstrust.sim

import io.gnsstrust.core.SimRandom
import io.gnsstrust.core.psiToCompass
import io.gnsstrust.model.*
import kotlin.math.*

/**
 * 传感器误差仿真。
 *
 * 这里的每一个误差源都是照着"让检测变难"的方向调的:
 *   - 城市峡谷里 GNSS 位置误差是【慢变 OU 过程】而不是白噪声:
 *     多径会让定位在几十米范围内以几十秒的尺度来回漂, 这和"缓慢欺骗"在
 *     任何单点上都几乎不可区分。这是本问题的真实难度来源。
 *   - IMU 有零偏不稳定性 + 标度因数误差, 积分会漂。
 *   - 轮速有脉冲量化误差, 低速时精度骤降, 且会打滑。
 *   - 气压计随天气缓慢漂移 (几十分钟几米), 所以不能当绝对高度用。
 *   - 磁罗盘在车体附近有硬铁偏差, 过金属结构时会突变。
 *
 * 关键: GNSS 的【多普勒速度】精度远高于【位置】精度, 且二者误差不相关。
 * 在真实城市峡谷里位置在漂但速度是准的 —— 这给检测器提供第一条独立线索,
 * 前提是攻击者没有同时伪造 Doppler。
 */

private class CellState { var rssi = -90.0 }

class SensorSim(
    val seed: Long,
    val platform: Platform,
) {
    private val R = SimRandom(seed * 977 + 11)

    // ---- 持久化的慢变误差状态
    private var mpE = 0.0; private var mpN = 0.0; private var mpAlt = 0.0
    private var ionoE = 0.0; private var ionoN = 0.0; private var ionoAlt = 0.0
    private var baroDrift = 0.0
    private var magBiasDeg = R.uniform(-8.0, 8.0)

    private val bg = doubleArrayOf(R.gauss(0.0, 0.0015), R.gauss(0.0, 0.0015), R.gauss(0.0, 0.0020))
    private val ba = doubleArrayOf(R.gauss(0.0, 0.06), R.gauss(0.0, 0.06), R.gauss(0.0, 0.09))
    private val sg = doubleArrayOf(1.0 + R.gauss(0.0, 0.004), 1.0 + R.gauss(0.0, 0.004), 1.0 + R.gauss(0.0, 0.006))
    private val yawDriftRate = R.gauss(0.0, 0.00012)

    private var slipUntil = -1.0
    private var lastSlipT = -999.0

    // 由场景注入
    var towers: List<Tower> = emptyList()
    var aps: List<Ap> = emptyList()
    var anchors: List<Anchor> = emptyList()
    private val cellStates = HashMap<Long, CellState>()

    /** 生成一帧的全部传感器 */
    fun frame(i: Int, tr: TruthState, att: AttackSample, dt: Double): SensorFrame {
        val t = tr.t
        val env = tr.env

        val gnss = buildGnss(i, tr, att, dt)
        val imu = buildImu(t, tr, dt)
        val wheel = if (platform == Platform.TRUCK) buildWheel(t, tr, dt) else null

        baroDrift += R.gauss(0.0, 0.010 * sqrt(dt))
        baroDrift *= (1.0 - dt / 3600.0)
        val baro = BaroObs(t, tr.altM + baroDrift + R.gauss(0.0, 0.09), 0.35)

        val magDisturbed = env == EnvClass.URBAN_CANYON || env == EnvClass.INDOOR_GARAGE
        val mag = if (platform == Platform.TRUCK) {
            val dist = if (magDisturbed) R.gauss(0.0, 6.0) else R.gauss(0.0, 1.2)
            MagObs(
                t, psiToCompass(tr.psi + Math.toRadians(magBiasDeg + dist)),
                3.0 + (if (magDisturbed) 8.0 else 0.0), magDisturbed
            )
        } else null

        val (flow, flowSigma) = buildFlow(t, tr)
        val cells = simCells(tr)
        val wifis = simWifi(tr)
        val ranges = simRanges(t, tr)

        return SensorFrame(
            t = t, platform = platform, gnss = gnss, imu = imu, wheel = wheel,
            baro = baro, mag = mag, flowVelMps = flow, flowSigmaMps = flowSigma,
            cells = cells, wifis = wifis, ranges = ranges,
            cmdAccelMps2 = if (platform == Platform.DRONE) tr.aLong else null,
            inGarage = env == EnvClass.INDOOR_GARAGE,
        )
    }

    // ------------------------------------------------------------ 独立测距

    /**
     * 与 GNSS 完全无关的第二把尺子。
     *
     * 卡车: 服务基站的 TA 只有主服务小区提供, 精度随距离变差; 车在隧道/地库
     *       时仍有蜂窝, 所以它能覆盖 GNSS 盲区 —— 也正因如此, 攻击者在
     *       "隧道里开始拉偏"这一招上会吃亏。
     * 无人机: C2 链路 TWR 每 1 Hz 一次, 取最近的地面站; 视距被楼挡时 (
     *       低空穿越城市峡谷) 会拒绝服务。
     *
     * 注意这里的 [sigma] 是工程上"诚实"的量级: 几十米。它没法单点定位,
     * 但足以在 100 s 的尺度上把 300 m 以上的漂移钉死。
     */
    private fun simRanges(t: Double, tr: TruthState): List<RangeObs> {
        val out = ArrayList<RangeObs>(3)
        if (platform == Platform.DRONE) {
            for (an in anchors) {
                if (an.kind != AnchorKind.C2_LINK && an.kind != AnchorKind.DEPOT_UWB) continue
                val dh = hypot(an.e - tr.e, an.n - tr.north)
                val slant = hypot(dh, an.altM - tr.altM)
                if (slant > 1600.0) continue
                // 城市峡谷低空: 楼体遮挡, 视距链路丢失
                val blocked = tr.env == EnvClass.URBAN_CANYON && tr.altM < 55.0
                if (blocked && an.kind == AnchorKind.C2_LINK) continue
                val sigma = if (an.kind == AnchorKind.DEPOT_UWB) 6.0 else 12.0 + slant * 0.012
                // TWR 的测距噪声是白噪声, 但多径带来慢变偏置
                val bias = 0.0
                out.add(RangeObs(an.id, slant + bias + R.gauss(0.0, sigma), sigma, an.kind))
                if (an.kind == AnchorKind.DEPOT_UWB) break
            }
            return out.take(2)
        }

        // 卡车: 只取主服务基站的 TA
        val tower = anchors.filter { it.kind == AnchorKind.CELL_TA }.minByOrNull {
            hypot(it.e - tr.e, it.n - tr.north)
        } ?: return out
        val d = hypot(tower.e - tr.e, tower.n - tr.north)
        val inTunnel = tr.env == EnvClass.TUNNEL
        // TA 在隧道里不可用 (泄漏电缆不提供定时提前)
        if (inTunnel) return out
        // TA 量化: 5G NR 单程定时提前步长折算约 39 m
        val step = 39.0
        val sigma = 28.0 + d * 0.02
        // 设备/天线时延的固定偏移在标定阶段已扣除 (否则会给测距引入系统性
        // 偏差, 让所有下游的 CUSUM 一直朝一个方向累积 —— 这是踩过的坑)
        val raw = d + R.gauss(0.0, sigma)
        val quantised = round(raw / step) * step
        out.add(RangeObs(tower.id, quantised, sigma + step / sqrt(12.0), AnchorKind.CELL_TA))
        return out
    }

    // ------------------------------------------------------------ GNSS

    private fun buildGnss(i: Int, tr: TruthState, att: AttackSample, dt: Double): GnssFix {
        val env = tr.env
        val blackout = att.jam || env == EnvClass.TUNNEL ||
            (env == EnvClass.INDOOR_GARAGE && R.bernoulli(0.55))
        if (blackout) {
            // 地库里偶尔漏进 2D 解 (只有 4 颗卫星)
            if (env == EnvClass.INDOOR_GARAGE && !att.jam && R.bernoulli(0.35)) {
                return degradedFix(tr)
            }
            return GnssFix.invalid(tr.t)
        }

        val (whiteSigma, ouSigma, ouTau) = when (env) {
            EnvClass.OPEN_SKY -> Triple(1.4, 0.7, 120.0)
            EnvClass.SUBURBAN -> Triple(2.2, 1.2, 90.0)
            EnvClass.URBAN_CANYON -> Triple(4.0, 7.5, 45.0)
            else -> Triple(6.0, 10.0, 30.0)
        }
        val a = exp(-dt / ouTau)
        val q = ouSigma * sqrt(1 - a * a)
        mpE = a * mpE + q * R.gauss()
        mpN = a * mpN + q * R.gauss()
        mpAlt = a * mpAlt + q * 1.6 * R.gauss()
        ionoE = (ionoE + R.gauss(0.0, 0.05 * sqrt(dt))).coerceIn(-1.5, 1.5)
        ionoN = (ionoN + R.gauss(0.0, 0.05 * sqrt(dt))).coerceIn(-1.5, 1.5)
        ionoAlt += R.gauss(0.0, 0.06 * sqrt(dt))

        val nVis = when (env) {
            EnvClass.OPEN_SKY -> 11; EnvClass.SUBURBAN -> 10
            EnvClass.URBAN_CANYON -> 6; else -> 5
        } + R.int(3) - 1
        val cno = DoubleArray(max(4, nVis))
        val prn = IntArray(cno.size)
        val elev = DoubleArray(cno.size)
        val baseCno = when (env) {
            EnvClass.OPEN_SKY -> 45.0; EnvClass.SUBURBAN -> 42.0
            EnvClass.URBAN_CANYON -> 39.0; else -> 33.0
        }
        // 仰角分布: 开阔/郊区有大量低仰角卫星; 峡谷里楼体把它们全挡住了。
        // 这是区分"峡谷 vs 郊区"最有物理意义的特征。
        val elevCut = when (env) {
            EnvClass.URBAN_CANYON -> 34.0; EnvClass.SUBURBAN -> 10.0; else -> 6.0
        }
        for (k in cno.indices) {
            elev[k] = R.uniform(elevCut, 88.0)
            cno[k] = baseCno + 6.0 * sin(elev[k] * PI / 180) + R.gauss(0.0, 2.0) + att.cnoShiftDb
            prn[k] = visiblePrn(env, k)
        }
        val agc = R.gauss(0.0, 1.2) + att.cnoShiftDb * 0.9

        var e = tr.e + mpE + ionoE + R.gauss(0.0, whiteSigma)
        var n = tr.north + mpN + ionoN + R.gauss(0.0, whiteSigma)
        var alt = tr.altM + mpAlt + ionoAlt + R.gauss(0.0, whiteSigma)

        val velSigma = when (env) {
            EnvClass.OPEN_SKY -> 0.055; EnvClass.SUBURBAN -> 0.09
            EnvClass.URBAN_CANYON -> 0.16; else -> 0.45
        }
        var vE = tr.vE + R.gauss(0.0, velSigma)
        var vN = tr.vN + R.gauss(0.0, velSigma)
        var course = if (tr.trueSpeedMps > 0.6) atan2(vN, vE) else tr.psi

        val reportedSigma = when (env) {
            EnvClass.OPEN_SKY -> 2.6; EnvClass.SUBURBAN -> 4.2
            EnvClass.URBAN_CANYON -> 11.0; else -> 25.0
        } * R.uniform(0.85, 1.2)

        var clockJump = 0.0
        var residual = when (env) {
            EnvClass.OPEN_SKY -> 1.1; EnvClass.SUBURBAN -> 1.8
            EnvClass.URBAN_CANYON -> 3.2; else -> 4.0
        } * R.uniform(0.8, 1.3)
        var speedSigma = velSigma
        var satCount = cno.size
        var rejected = 0
        var prnOut = prn

        if (att.active) {
            e += att.dE
            n += att.dN
            alt += att.dAlt
            if (att.velE != null && att.velN != null) {
                vE = att.velE + R.gauss(0.0, velSigma * 0.6)
                vN = att.velN + R.gauss(0.0, velSigma * 0.6)
                course = att.course ?: atan2(vN, vE)
            }
            speedSigma *= att.speedSigmaMult
            clockJump = att.clockJumpM
            residual = att.residualRmsM * R.uniform(0.8, 1.2)
            satCount = (cno.size * att.spoofedFrac).roundToInt().coerceAtLeast(4)
            // 全接管: 星座整体"换血" —— 天线只看得见假星座, 真实星座被压制
            if (att.spoofedFrac >= 0.99) {
                prnOut = IntArray(cno.size) { 200 + spoofPrnOffset + it }
                rejected = 0
            } else {
                rejected = (cno.size * (1 - att.spoofedFrac)).roundToInt()
                prnOut = prn.copyOf(satCount)
            }
        }

        val speed = if (tr.trueSpeedMps < 0.15 && !att.active) 0.0 else hypot(vE, vN)
        return GnssFix(
            t = tr.t, valid = true,
            eM = e, nM = n, altM = alt,
            posSigmaM = reportedSigma, vertSigmaM = reportedSigma * 1.7,
            speedMps = speed, speedSigmaMps = speedSigma,
            courseRad = course, courseSigmaRad = if (speed < 0.5) PI / 2 else 0.05,
            sats = satCount, fixType = FixType.THREE_D, prn = prnOut,
            cnoDbHz = cno, elevDeg = elev, agcDb = agc,
            clockJumpM = clockJump, residualRmsM = residual, rejectedSats = rejected,
        )
    }

    private fun degradedFix(tr: TruthState): GnssFix = GnssFix(
        t = tr.t, valid = true,
        eM = tr.e + R.gauss(0.0, 30.0), nM = tr.north + R.gauss(0.0, 30.0), altM = 0.0,
        posSigmaM = 35.0, vertSigmaM = 60.0,
        speedMps = 0.0, speedSigmaMps = 2.0, courseRad = 0.0, courseSigmaRad = PI / 2,
        sats = 4, fixType = FixType.TWO_D, prn = intArrayOf(7, 13, 21, 29),
        cnoDbHz = doubleArrayOf(28.0, 26.0, 25.0, 24.0),
        elevDeg = doubleArrayOf(40.0, 35.0, 30.0, 25.0),
        agcDb = -3.0, clockJumpM = 0.0, residualRmsM = 9.0, rejectedSats = 0,
    )

    /**
     * 可见卫星 PRN 集合。同一颗卫星会在天上待几个小时, 所以正常运行时
     * 每帧的集合几乎不变 (只有升落时缓慢轮换)。欺骗者接管天线后必须
     * 自己生成一整套星座 —— 换血是瞬时的, 这是最强的射频指纹。
     */
    private var prnRoster = IntArray(0)
    private var rosterEpoch = -999

    private fun visiblePrn(env: EnvClass, k: Int): Int {
        if (prnRoster.isEmpty()) {
            prnRoster = IntArray(14) { 2 + it * 3 }
            rosterEpoch = 0
            // 每 20 分钟轮换一颗 (模拟卫星升落)
            val lcg = (seed * 6364136223846793005L).ushr(24).toInt()
            prnRoster = IntArray(14) { ((lcg + it * 17) % 32 + 1) }.distinct().toIntArray()
        }
        return prnRoster[k % prnRoster.size]
    }

    private val spoofPrnOffset: Int get() = ((seed % 97).toInt() * 11 + 40)

    // ------------------------------------------------------------ IMU

    private fun buildImu(t: Double, tr: TruthState, dt: Double): ImuObs {
        val g = 9.80665
        val speed = tr.trueSpeedMps
        // 姿态: 车体侧倾/俯仰
        val roll = if (platform == Platform.DRONE)
            atan2(speed * tr.yawRate, g) * 0.55
        else atan2(speed * tr.yawRate, g) * 0.62
        val pitch = if (platform == Platform.DRONE) (tr.aLong / g).coerceIn(-0.35, 0.35) * 0.75 else 0.0

        // 车体系 (x 前, y 左, z 上) 中的重力分量
        val gBodyX = -g * sin(pitch)
        val gBodyY = g * sin(roll) * cos(pitch)
        val gBodyZ = g * cos(roll) * cos(pitch)

        // 运动加速度转到机体系
        val aBodyX = tr.aLong
        val aBodyY = tr.aLat

        // 振动幅度: 这是**低通之后**残余在解算里的量级, 不是原始 IMU 输出。
        // 原始输出会被 50-100 Hz 的机械振动主导, 车机都会先做带通/低通。
        val vibAmp = if (platform == Platform.DRONE) 0.30 else 0.055 + 0.012 * speed
        val vibFreq = if (platform == Platform.DRONE) 95.0 else 11.0
        val vibA = vibAmp * (0.4 * sin(2 * PI * vibFreq * t) + 0.35 * R.gauss())
        val vibB = vibAmp * (0.4 * sin(2 * PI * vibFreq * t + 1.1) + 0.35 * R.gauss())

        for (k in 0..2) ba[k] += R.gauss(0.0, 0.0016 * sqrt(dt))
        for (k in 0..2) bg[k] += R.gauss(0.0, 0.000012 * sqrt(dt))

        return ImuObs(
            t = t,
            ax = sg[0] * (aBodyX + gBodyX) + ba[0] + vibA + R.gauss(0.0, 0.05),
            ay = sg[1] * (aBodyY + gBodyY) + ba[1] + vibB + R.gauss(0.0, 0.05),
            az = sg[2] * (gBodyZ + tr.aVert) + ba[2] + vibA * 0.5 + R.gauss(0.0, 0.07),
            gx = bg[0] + R.gauss(0.0, 0.0025),
            gy = bg[1] + R.gauss(0.0, 0.0025),
            gz = sg[2] * tr.yawRate + bg[2] + yawDriftRate + R.gauss(0.0, 0.0025),
        )
    }

    // ------------------------------------------------------------ 轮速

    private fun buildWheel(t: Double, tr: TruthState, dt: Double): WheelObs {
        if (t > lastSlipT + 8.0 && abs(tr.aLong) > 2.2 && R.bernoulli(0.012)) {
            slipUntil = t + R.uniform(0.3, 1.2); lastSlipT = t
        }
        val slipping = t < slipUntil
        val trueV = tr.trueSpeedMps
        val slipFactor = if (slipping) 1.0 + R.uniform(0.10, 0.22) * sign(tr.aLong + 1e-9) else 1.0
        val tickM = 0.175
        val vQuant = when {
            trueV < 0.2 -> 0.0
            slipping -> trueV * slipFactor
            else -> round(trueV / tickM) * tickM
        }
        return WheelObs(
            t = t, speedMps = vQuant,
            quantSigmaMps = max(0.02, if (trueV < 1.0) tickM * 0.9 else tickM * 0.28),
            slip = slipping,
        )
    }

    // ------------------------------------------------------------ 光流 (无人机)

    private fun buildFlow(t: Double, tr: TruthState): Pair<DoubleArray?, Double> {
        if (platform != Platform.DRONE) return null to 0.0
        val alt = tr.altM
        val textureLost = sin(t / 37.0) > 0.82 || alt > 110.0
        return if (textureLost) {
            null to 3.0
        } else {
            val sigma = 0.35 + 0.0022 * alt
            val scaleErr = 1.0 + R.gauss(0.0, 0.008)
            val h = max(alt, 8.0)
            // 光流传感器给出的是**车体坐标系**下的对地速度 (前/左), 不是世界系。
            // 这是真实传感器的工作方式; 若按世界系给, 滤波器就白拿了航向。
            val c = cos(tr.psi); val s = sin(tr.psi)
            val vFwd = tr.vE * c + tr.vN * s
            val vLeft = -tr.vE * s + tr.vN * c
            doubleArrayOf(
                vFwd * scaleErr + R.gauss(0.0, sigma),
                vLeft * scaleErr + R.gauss(0.0, sigma),
            ) to sigma
        }
    }

    // ------------------------------------------------------------ 蜂窝 / WiFi

    private fun simCells(tr: TruthState): List<CellObs> {
        val out = ArrayList<CellObs>(4)
        for (tw in towers) {
            val d = hypot(tw.x - tr.e, tw.y - tr.north)
            val rssi = tw.pTxDbm - 20 * log10(max(d, 5.0)) - 8.0 + R.gauss(0.0, 2.5)
            if (rssi < -112) { cellStates.remove(tw.id); continue }
            val st = cellStates.getOrPut(tw.id) { CellState() }
            st.rssi = 0.7 * st.rssi + 0.3 * rssi
            out.add(CellObs(tw.id, st.rssi, if (tw.h > 40) d else null))
        }
        out.sortByDescending { it.rssiDbm }
        return out.take(2)
    }

    private fun simWifi(tr: TruthState): List<WifiObs> {
        if (platform == Platform.DRONE && tr.altM > 60) return emptyList()
        val out = ArrayList<WifiObs>()
        for (ap in aps) {
            val d = hypot(ap.x - tr.e, ap.y - tr.north)
            if (d > 160.0) continue
            val pathLoss = 20 * log10(max(d, 2.0)) + 40.0
            val rssi = ap.pTxDbm - pathLoss + R.gauss(0.0, 5.0) - 45.0
            if (rssi < -88) continue
            out.add(WifiObs(ap.bssid, rssi, ap.freq))
        }
        out.sortByDescending { it.rssiDbm }
        return out.take(4)
    }
}

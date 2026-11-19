package io.gnsstrust.detect

import io.gnsstrust.model.*
import kotlin.math.*

/**
 * 从观测量反推信号环境。
 *
 * 为什么不能靠外部配置: 检测门槛的每一档都按环境缩放。如果"现在在峡谷里"
 * 这个判断本身能被攻击者影响 (比如他压低 C/N0 让你以为在峡谷, 从而放宽门槛),
 * 整套阈值体系就失效了。所以这里只用**接收机射频层**指标。
 *
 * 两个工程要点:
 *  1. 特征先做 EWMA 平滑再分类。逐帧的卫星数/残差抖动很大, 直接比较会让
 *     分类目标每帧跳变, 滞后计时器被无限重置 —— 这个坑在开发中踩过。
 *  2. 滞后按**时间**实现, 不按帧数。按帧数会让行为随采样率悄悄改变。
 */
class EnvClassifier {

    private var env: EnvClass? = null
    private var candidate: EnvClass? = null
    private var candidateSince = Double.NaN
    private var lastT = Double.NaN
    var confidence = 0.0
        private set

    // EWMA 特征
    private var sSats = 0.0
    private var sCno = 0.0
    private var sResid = 0.0
    private var sLowElev = 0.0
    private var nSeen = 0

    fun classify(f: SensorFrame): EnvClass {
        val g = f.gnss
        val dt = if (lastT.isNaN()) 0.1 else (f.t - lastT).coerceIn(1e-3, 1.0)
        lastT = f.t

        if (!g.valid) {
            confidence = 0.9
            if (f.inGarage && env != EnvClass.INDOOR_GARAGE)
                return hold(EnvClass.INDOOR_GARAGE, f.t, force = true)
            return env ?: EnvClass.URBAN_CANYON
        }

        val medCno = median(g.cnoDbHz)
        val lowElev = if (g.elevDeg.isEmpty()) 1.0 else
            g.elevDeg.count { it < 25.0 }.toDouble() / g.elevDeg.size

        // 时间常数 2 s 的 EWMA
        val a = 1.0 - exp(-dt / 2.0)
        sSats = if (nSeen == 0) g.sats.toDouble() else sSats + a * (g.sats - sSats)
        sCno = if (nSeen == 0) medCno else sCno + a * (medCno - sCno)
        sResid = if (nSeen == 0) g.residualRmsM else sResid + a * (g.residualRmsM - sResid)
        sLowElev = if (nSeen == 0) lowElev else sLowElev + a * (lowElev - sLowElev)
        nSeen++

        // 峡谷与郊区的区分不能只看 C/N0 —— 峡谷里卫星被"照亮"得多, 有时
        // C/N0 反而更高。真正有区分度的是: 卫星数、低仰角星是否被遮挡、
        // 以及接收机残差。三个条件同时满足才算郊区。
        val target = when {
            f.inGarage -> EnvClass.INDOOR_GARAGE
            sSats >= 8.5 && sCno >= 42.0 && sResid < 2.2 && sLowElev < 0.08 -> EnvClass.OPEN_SKY
            sSats >= 8.0 && sLowElev >= 0.15 && sResid < 3.4 -> EnvClass.SUBURBAN
            else -> EnvClass.URBAN_CANYON
        }
        val out = hold(target, f.t)
        confidence = when {
            out != target -> 0.6
            (out == EnvClass.SUBURBAN && sResid > 4.0) -> 0.7
            else -> 0.93
        }
        return out
    }

    private fun hold(target: EnvClass, t: Double, force: Boolean = false): EnvClass {
        val cur = env
        if (cur == null) { env = target; return target }
        if (target == cur) { candidate = null; candidateSince = Double.NaN; return cur }
        if (force) { env = target; candidate = null; candidateSince = Double.NaN; return target }
        if (target != candidate) { candidate = target; candidateSince = t }
        val dwell = if (cur == EnvClass.INDOOR_GARAGE || target == EnvClass.INDOOR_GARAGE) 3.0 else 10.0
        if (t - candidateSince >= dwell) {
            env = target; candidate = null; candidateSince = Double.NaN
            return target
        }
        return cur
    }

    fun current(): EnvClass? = env

    private fun median(a: DoubleArray): Double {
        if (a.isEmpty()) return 0.0
        val s = a.sorted()
        return s[s.size / 2]
    }
}

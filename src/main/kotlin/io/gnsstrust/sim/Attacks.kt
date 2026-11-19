package io.gnsstrust.sim

import io.gnsstrust.model.EnvClass
import io.gnsstrust.model.Platform
import io.gnsstrust.model.TruthState
import kotlin.math.*

/**
 * 攻击模型。
 *
 * 这里刻意把"攻击者有多强"参数化, 而不是写死几个脚本 ——
 * 因为整个模块要回答的核心问题是:
 *   在攻击者选择【最优拉偏速率】的情况下, 他最多能在被发现前把位置挪多远?
 *
 * 一个重要区分:
 *  - 一致欺骗 (consistent): 攻击者接管了全部卫星信号, 接收机解算出的位置/速度/航向
 *    完全自洽地对应一条假轨迹。这是"有能力的攻击者"的默认形态。
 *    接收机自身的一切自检 (残差、RAIM、Doppler 一致性) 都看不出来。
 *    唯一能发现它的是【车上的独立传感器】与【独立的绝对参照】。
 *  - 不一致欺骗: 只伪造位置, 不伪造 Doppler。现实中多见于旧式接收机或
 *    部分卫星被捕获。接收机残差会升高, 更容易被抓。
 *
 * 检测模块必须对前者成立 —— 后者只是顺带能抓。
 */

data class AttackSample(
    val active: Boolean,
    val dE: Double,
    val dN: Double,
    val dAlt: Double,
    /** null 表示"速度未伪造", 即沿用真实速度 */
    val velE: Double?,
    val velN: Double?,
    val course: Double?,
    val speedSigmaMult: Double,
    val cnoShiftDb: Double,
    /** 被伪造的卫星比例 [0,1]; 1.0 为全接管 */
    val spoofedFrac: Double,
    val clockJumpM: Double,
    val residualRmsM: Double,
    val jam: Boolean,
    val kind: String,
) {
    companion object {
        val CLEAN = AttackSample(
            false, 0.0, 0.0, 0.0, null, null, null, 1.0, 0.0, 0.0, 0.0, 0.0, false, "NONE"
        )
    }
}

interface AttackModel {
    val kind: String
    fun at(i: Int, t: Double, tr: TruthState, track: TruthTrack): AttackSample
    /** 攻击者开始活动的时刻, 用于评估 (NaN 表示全程无攻击) */
    val onset: Double
}

// ------------------------------------------------------------------ 无攻击

object NoAttack : AttackModel {
    override val kind = "NONE"
    override val onset = Double.NaN
    override fun at(i: Int, t: Double, tr: TruthState, track: TruthTrack) = AttackSample.CLEAN
}

// ------------------------------------------------------------------ 缓慢拉偏

/**
 * 经典"牵引式"欺骗: 从真实位置出发, 以 [rateMps] 匀速把位置拉向某方向,
 * 拉满 [maxOffsetM] 后停住 (停住后速度恢复正常, 位置偏移保留)。
 *
 * @param consistent true = 同时伪造 Doppler, 使接收机速度与假轨迹自洽 (强攻击者)
 * @param rotRadPerSec 拉偏方向缓慢旋转 (让偏移在几何上也像一条曲线)
 * @param gate 只在满足条件的环境里拉 (环境感知攻击者), 其余时间保持当前偏移
 */
class GradualDrift(
    override val onset: Double,
    val rateMps: Double,
    val maxOffsetM: Double,
    val consistent: Boolean,
    val rotRadPerSec: Double = 0.0,
    val dirDeg: Double = 0.0,
    val altRateMps: Double = 0.0,
    val gate: ((EnvClass) -> Boolean)? = null,
) : AttackModel {
    override val kind: String = when {
        gate != null -> "DRIFT_ENV_AWARE"
        consistent -> "DRIFT_CONSISTENT"
        else -> "DRIFT_POS_ONLY"
    }

    private var frozenMag = 0.0
    private var frozenAlt = 0.0
    private var lastT = Double.NaN

    override fun at(i: Int, t: Double, tr: TruthState, track: TruthTrack): AttackSample {
        if (t < onset) return AttackSample.CLEAN
        val gated = gate?.let { it(tr.env) } ?: true
        val dtStep = if (lastT.isNaN()) 0.0 else (t - lastT)
        lastT = t

        if (gated) {
            frozenMag = min(maxOffsetM, frozenMag + rateMps * dtStep)
            frozenAlt = frozenAlt + altRateMps * dtStep
        }
        val age = t - onset
        val theta = Math.toRadians(dirDeg) + rotRadPerSec * age
        val dE = frozenMag * cos(theta)
        val dN = frozenMag * sin(theta)
        // 偏移对时间的导数 (含方向旋转项) —— 一致欺骗必须把它加进伪造的速度里
        val saturating = frozenMag < maxOffsetM - 1e-9 && gated
        val dMagDt = if (saturating) rateMps else 0.0
        val dthetaDt = if (saturating) rotRadPerSec else 0.0
        val vdE = dMagDt * cos(theta) - frozenMag * dthetaDt * sin(theta)
        val vdN = dMagDt * sin(theta) + frozenMag * dthetaDt * cos(theta)

        val velE: Double?
        val velN: Double?
        val course: Double?
        if (consistent) {
            velE = tr.vE + vdE
            velN = tr.vN + vdN
            course = if (hypot(velE, velN) > 0.3) atan2(velN, velE) else tr.psi
        } else {
            velE = null; velN = null; course = null
        }

        return AttackSample(
            active = true,
            dE = dE, dN = dN, dAlt = frozenAlt,
            velE = velE, velN = velN, course = course,
            speedSigmaMult = 1.0,
            cnoShiftDb = 2.5,
            spoofedFrac = 1.0,
            clockJumpM = 0.0,
            residualRmsM = if (consistent) 0.9 else 3.5,
            jam = false,
            kind = kind,
        )
    }

    fun offsetAt(t: Double): Double {
        if (t < onset) return 0.0
        // 近似: 门控不影响总量上限
        return min(maxOffsetM, rateMps * (t - onset))
    }
}

// ------------------------------------------------------------------ 跳变接管

class JumpTakeover(
    override val onset: Double,
    val dE: Double,
    val dN: Double,
    val dAlt: Double = 0.0,
) : AttackModel {
    override val kind = "JUMP"
    override fun at(i: Int, t: Double, tr: TruthState, track: TruthTrack): AttackSample {
        if (t < onset) return AttackSample.CLEAN
        return AttackSample(
            true, dE, dN, dAlt,
            tr.vE, tr.vN, tr.psi, 1.0, 4.0, 1.0, 0.35, 0.8, false, kind
        )
    }
}

// ------------------------------------------------------------------ 冻结位置 (meaconing)

/**
 * 中继式 ("meaconing"): 把过去的信号原样转发, 接收机以为自己在原地。
 * 位置冻结在 [onset] 时刻的点, 速度报告为 0。
 */
class FrozenPosition(override val onset: Double) : AttackModel {
    override val kind = "FROZEN"
    override fun at(i: Int, t: Double, tr: TruthState, track: TruthTrack): AttackSample {
        if (t < onset) return AttackSample.CLEAN
        val j = track.indexAt(onset)
        return AttackSample(
            true,
            track.e[j] - tr.e, track.nArr[j] - tr.north, track.alt[j] - tr.altM,
            0.0, 0.0, tr.psi, 1.0, 1.5, 1.0, 0.0, 1.1, false, kind
        )
    }
}

// ------------------------------------------------------------------ 重放

/** 重放 [delayS] 秒前的真实轨迹: 位置、速度、航向全部自洽, 只是"晚"了。 */
class Replay(override val onset: Double, val delayS: Double) : AttackModel {
    override val kind = "REPLAY"
    override fun at(i: Int, t: Double, tr: TruthState, track: TruthTrack): AttackSample {
        if (t < onset) return AttackSample.CLEAN
        val j = track.indexAt(t - delayS)
        return AttackSample(
            true,
            track.e[j] - tr.e, track.nArr[j] - tr.north, track.alt[j] - tr.altM,
            track.vE[j], track.vN[j], track.psi[j], 1.0, 1.0, 1.0, 0.0, 0.9, false, kind
        )
    }
}

// ------------------------------------------------------------------ 幽灵航线

/**
 * "偷跑私活"的主力形态: 车真实停/慢行在别处, GNSS 却报告它正沿着
 * 计划航线以正常车速前进。调度员看到的是一条完美合规的轨迹。
 */
class GhostRoute(
    override val onset: Double,
    val ghostSpeedMps: Double,
    val path: Path,
) : AttackModel {
    override val kind = "GHOST_ROUTE"
    override fun at(i: Int, t: Double, tr: TruthState, track: TruthTrack): AttackSample {
        if (t < onset) return AttackSample.CLEAN
        val s0 = track.sAlong[track.indexAt(onset)]
        val s = min(path.total - 1.0, s0 + ghostSpeedMps * (t - onset))
        val p = path.at(s)
        val vE = p[2] * ghostSpeedMps
        val vN = p[3] * ghostSpeedMps
        return AttackSample(
            true, p[0] - tr.e, p[1] - tr.north, 0.0,
            vE, vN, atan2(vN, vE), 1.0, 2.0, 1.0, 0.0, 0.8, false, kind
        )
    }
}

// ------------------------------------------------------------------ 纯压制

class JamOnly(override val onset: Double) : AttackModel {
    override val kind = "JAM"
    override fun at(i: Int, t: Double, tr: TruthState, track: TruthTrack): AttackSample {
        if (t < onset) return AttackSample.CLEAN
        return AttackSample.CLEAN.copy(jam = true, active = false, kind = kind)
    }
}

// ------------------------------------------------------------------ 攻击参数枚举

data class ScenarioSpec(
    val name: String,
    val platform: Platform,
    val attack: AttackModel,
    val seed: Long,
    val durationS: Double,
    /** 冷启动: 前 [coldStartS] 秒没有可靠参照 (刚上电/刚出地库) */
    val coldStart: Boolean = false,
    val coldStartS: Double = 30.0,
    /** 全程静止 (停在路边, 伪装成"车在装货") */
    val stationaryWholeTime: Boolean = false,
    /** 真值轨迹里刻意安排 GNSS 失锁的窗口, 用于验证"失锁期间不能判" */
    val forceOutage: List<ClosedFloatingPointRange<Double>> = emptyList(),
)

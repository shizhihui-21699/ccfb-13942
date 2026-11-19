package io.gnsstrust.model

/**
 * 传感器与场景的数据契约。
 *
 * 设计原则: 每个观测量都带"它自己声称的精度"。检测器要做的事情就是
 * 看这些声称的精度与其它来源是否自洽 —— 欺骗者的破绽往往不在数值,
 * 而在"数值与它自己的精度声明不匹配"。
 */

enum class Platform { TRUCK, DRONE }

enum class FixType { NONE, TWO_D, THREE_D, RTK_FLOAT, RTK_FIXED }

/**
 * 信号环境。它决定三件事:
 *  1. GNSS 的噪声模型 (开阔天空 vs 城市峡谷, 差一个量级还多)
 *  2. GNSS 是否可用 (隧道/地库)
 *  3. 是否有独立的绝对参照 (蜂窝/WiFi 指纹)
 *
 * 注意: 真实系统里这个枚举不该由人工输入, 而应由 [EnvClassifier] 从数据推断。
 */
enum class EnvClass(val gnssPosSigmaM: Double, val gnssVelSigmaMps: Double, val gnssUsable: Boolean) {
    OPEN_SKY(2.5, 0.05, true),
    SUBURBAN(4.0, 0.10, true),
    URBAN_CANYON(12.0, 0.35, true),
    TUNNEL(999.0, 999.0, false),
    INDOOR_GARAGE(999.0, 999.0, false);

    val hasDirectView: Boolean get() = this == OPEN_SKY || this == SUBURBAN
}

/**
 * 一条卫星定位解。
 *
 * 位置用场景局部 ENU 直角坐标 [eM]/[nM] (米)。真实系统在接入层把
 * 纬度/经度转成局部 ENU 一次 (见 [io.gnsstrust.core.LocalFrame]),
 * 模块内部全程不再碰经纬度: 协方差矩阵因此有物理量纲, 也不会出现
 * "大数相减" 的精度损失。
 *
 * 关键点: [posSigmaM] 是接收机自己报的精度 (通常来自 HDOP/uere, 乐观),
 * 而 [trueNoiseSigmaM] 只在仿真里存在, 用来检验"检测器是否相信接收机的自我声明"。
 */
data class GnssFix(
    val t: Double,
    val valid: Boolean,
    val eM: Double,
    val nM: Double,
    val altM: Double,
    val posSigmaM: Double,
    val vertSigmaM: Double,
    val speedMps: Double,
    val speedSigmaMps: Double,
    val courseRad: Double,
    val courseSigmaRad: Double,
    val sats: Int,
    val fixType: FixType,
    /** 参与解算的卫星 PRN。欺骗接管会让整片星座"瞬间换血", 这是很强的指纹 */
    val prn: IntArray,
    /** 每颗可见卫星的载噪比 dB-Hz */
    val cnoDbHz: DoubleArray,
    /** 每颗卫星的仰角 (度), 与 cno 一一对应 */
    val elevDeg: DoubleArray,
    val agcDb: Double,
    /** 接收机报告的钟差跳变 (米), 对欺骗/重放很敏感 */
    val clockJumpM: Double,
    /** 位置解残差 RMS (米), 接收机已用的合法观测量一致性 */
    val residualRmsM: Double,
    /** 被接收机拒绝的卫星数 */
    val rejectedSats: Int,
) {
    companion object {
        fun invalid(t: Double) = GnssFix(
            t, false, 0.0, 0.0, 0.0, 999.0, 999.0, 0.0, 999.0, 0.0, 999.0,
            0, FixType.NONE, IntArray(0), DoubleArray(0), DoubleArray(0),
            0.0, 0.0, 999.0, 0
        )

        /** 供报告层展示: 从 ENU 转回经纬度 */
        fun latLon(f: GnssFix, frame: io.gnsstrust.core.LocalFrame): DoubleArray =
            frame.toLatLon(f.eM, f.nM)
    }
}

/** 轮速 / 里程计 (来自 CAN 总线, 攻击者无法通过射频篡改) */
data class WheelObs(
    val t: Double,
    val speedMps: Double,
    /** 脉冲量化: 低速时精度骤降 */
    val quantSigmaMps: Double,
    /** 打滑/ABS 介入标记 */
    val slip: Boolean,
)

/** 六轴 IMU, 车体坐标系 (x 前, y 右, z 下) */
data class ImuObs(
    val t: Double,
    val ax: Double, val ay: Double, val az: Double,
    val gx: Double, val gy: Double, val gz: Double,
)

data class BaroObs(val t: Double, val altM: Double, val sigmaM: Double)

/** 磁强计/电子罗盘给出的航向 */
data class MagObs(val t: Double, val headingDeg: Double, val sigmaDeg: Double, val disturbed: Boolean)

/** 蜂窝小区观测 */
data class CellObs(val cid: Long, val rssiDbm: Double, val timingAdvanceM: Double?)

/** WiFi 接入点观测 */
data class WifiObs(val bssid: Long, val rssiDbm: Double, val freqMhz: Int)

/**
 * 固定锚点。攻击者能骗过 GNSS, 但骗不过这些:
 *  - 蜂窝基站的定时提前量 (TA): 粗 (σ~50-120 m) 但绝对, 且用的是完全不同的频段
 *  - 无人机的 C2 遥控链路: 双向测距 (two-way ranging), σ~15-40 m,
 *    用的是 2.4/5.8 GHz 数传, GNSS 欺骗器碰不到
 *  - 仓库/场站的 UWB 或 WiFi 指纹: 停靠时精度极高
 * 它们共同构成"长时间尺度上不能无限漂"的物理依据 —— 没有它们,
 * 任何慢速拉偏在数学上都与惯导航向漂移不可区分。
 */
data class Anchor(val id: Long, val e: Double, val n: Double, val kind: AnchorKind)

enum class AnchorKind { CELL_TA, C2_LINK, DEPOT_UWB }

/** 到某个固定锚点的距离观测 (GNSS 无关) */
data class RangeObs(
    val anchorId: Long,
    val rangeM: Double,
    val sigmaM: Double,
    val kind: AnchorKind,
    /** 该观测是否可用; 视距被遮挡时为 false */
    val los: Boolean = true,
)

/**
 * 一个 100 ms 的传感器帧。检测器与滤波器只消费这个结构,
 * 所以把仿真换成真实车机数据不需要改检测逻辑。
 */
data class SensorFrame(
    val t: Double,
    val platform: Platform,
    val gnss: GnssFix,
    val imu: ImuObs,
    val wheel: WheelObs?,
    val baro: BaroObs,
    val mag: MagObs?,
    /** 光流/VIO 给出的对地速度 (无人机专用; 纹理贫乏或过高时会失效) */
    val flowVelMps: DoubleArray?,
    val flowSigmaMps: Double,
    val cells: List<CellObs>,
    val wifis: List<WifiObs>,
    /** 到固定锚点的独立测距 (不含 GNSS 信息) */
    val ranges: List<RangeObs>,
    /** 任务规划器下发的指令加速度 (车体前向), 仅无人机有 */
    val cmdAccelMps2: Double?,
    /** 地图上下文: 是否在地库里 (来自地图匹配, 不含 GNSS) */
    val inGarage: Boolean,
)

/** 仿真真值 —— 检测器绝对看不到, 只给评估用 */
data class TruthState(
    val t: Double,
    val e: Double, val north: Double,
    val vE: Double, val vN: Double,
    val psi: Double,
    val altM: Double,
    val env: EnvClass,
    val trueSpeedMps: Double,
    val yawRate: Double,
    /** 车体坐标下的加速度: 前向 / 左向 / 天向 */
    val aLong: Double = 0.0,
    val aLat: Double = 0.0,
    val aVert: Double = 0.0,
    val distanceTravelled: Double = 0.0,
)

/** 攻击者叠加在 GNSS 上的偏移 (评估用) */
data class AttackOffset(
    val t: Double,
    val dEast: Double,
    val dNorth: Double,
    val dAlt: Double,
    val active: Boolean,
    val kind: String,
) {
    val horizontal: Double get() = kotlin.math.hypot(dEast, dNorth)
}

data class ScenarioMeta(
    val name: String,
    val platform: Platform,
    val attackKind: String,
    val seed: Long,
    val durationS: Double,
    val truth: List<TruthState>,
    val attack: List<AttackOffset>,
    val frames: List<SensorFrame>,
    /** 场景中是否包含 GNSS 失锁区间 (隧道/地库) */
    val outageWindows: List<ClosedFloatingPointRange<Double>>,
    val jammedWindows: List<ClosedFloatingPointRange<Double>> = emptyList(),
    /** 地图: 攻击者无法通过射频篡改, 是少数"不可欺骗"的参照之一 */
    val roads: io.gnsstrust.sim.RoadNetwork? = null,
    /** 任务走廊 (无人机航线的计划容差带); null 表示不检查 */
    val corridor: List<DoubleArray>? = null,
    val corridorHalfWidthM: Double = Double.NaN,
    /** 场景真实包含的攻击时段 (评估用) */
    val attackWindows: List<ClosedFloatingPointRange<Double>> = emptyList(),
)

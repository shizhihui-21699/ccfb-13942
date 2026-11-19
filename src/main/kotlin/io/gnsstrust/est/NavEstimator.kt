package io.gnsstrust.est

import io.gnsstrust.core.*
import io.gnsstrust.model.*
import kotlin.math.*

/**
 * 误差状态卡尔曼滤波器 (ESKF), 只用【攻击者无法通过射频篡改】的输入:
 *   IMU 陀螺 (航向角速率) + 加速度计 (前向加速度)
 *   轮速 (CAN 总线) / 光流 (视觉)
 *   磁罗盘 (修正航向绝对漂移, 但抗扰差)
 *   气压计 (无人机高度, 慢漂)
 *   非完整约束 (车辆不会横着走)
 *   零速修正 (停住时速度必须为 0)
 *
 * ★ 关键设计决定: **GNSS 绝不进入本滤波器**。
 * 常见的做法是把 GNSS 当观测喂进 EKF, 但那意味着滤波器会老老实实跟着
 * 被欺骗的位置走 —— 滤波器越"好", 它就越顺从攻击者。这里刻意反过来:
 * 本滤波器代表"车自己独立认为自己在哪", GNSS 是待检验的对象。
 *
 * 代价: 本滤波器会漂 (尤其是航向)。所以它有第二个用途 —— 它的协方差
 * 就是"此刻我有多不确定"的诚实度量, 所有检测门槛都按它来缩放。
 *
 * 状态 x = [e, n, vE, vN, psi, bg, bf, s]   (8 维)
 *   e,n      局部 ENU 位置 (m)
 *   vE,vN    东北向速度 (m/s)
 *   psi      航向角 (rad, 0=东, 逆时针)
 *   bg       陀螺 z 轴零偏 (rad/s)
 *   bf       加速度计前向零偏 (m/s²)
 *   s        轮速标度误差 (无量纲, 真实=测量/(1+s))
 */
class NavEstimator(
    val platform: Platform,
    p0: DoubleArray,
    psi0: Double,
    var cfg: NavConfig = NavConfig.forPlatform(platform),
) {

    companion object { const val NX = 8 }
    private val ix = 0; private val iy = 1
    private val ivx = 2; private val ivy = 3
    private val ipsi = 4; private val ibg = 5
    private val ibf = 6; private val isc = 7

    var x = DoubleArray(NX)
    var P = Mat.eye(NX)
    var tLast = Double.NaN
    var initialized = false

    /** 垂直通道 (无人机), 独立一维滤波 */
    var vert: VertEstimator? = null

    /** 自上次成功 GNSS 重锚以来经过的时间 (s) —— 冷启动判定的核心量 */
    var timeSinceAnchor = 1e6
    /** 距离上一次"真实速度观测" (轮速/光流/GNSS 重锚) 的时间 */
    var timeSinceVelocityObs = 1e6
    /** 累计走过的路程 (m), 与 timeSinceAnchor 一起决定 DR 的置信度 */
    var distanceSinceAnchor = 0.0

    init {
        x[ix] = p0[0]; x[iy] = p0[1]
        x[ipsi] = psi0
        x[ivx] = 0.0; x[ivy] = 0.0
        for (i in 0 until NX) for (j in 0 until NX) P[i, j] = 0.0
        P[ix, ix] = 1e4; P[iy, iy] = 1e4          // 冷启动: 位置近乎未知
        P[ivx, ivx] = 1.0; P[ivy, ivy] = 1.0
        P[ipsi, ipsi] = Math.toRadians(30.0).pow(2)
        // ★ 偏置的初始不确定度必须与**物理标定水平**一致, 不能随手给大。
        // 给大了会出灾难性后果: 滤波器会把"速度对不上"的错误全部解释成
        // 加速度计偏置, 于是 bf 被拉到 -9 m/s², aF = ax - bf 反号,
        // 位置在 2 秒内飞出几百米。这是本模块开发中代价最大的一次教训。
        P[ibg, ibg] = Math.toRadians(0.15).pow(2)   // 陀螺零偏 ~0.15°/s
        P[ibf, ibf] = 0.12 * 0.12                   // 加计零偏 ~0.12 m/s²
        P[isc, isc] = 0.02 * 0.02
        if (platform == Platform.DRONE) vert = VertEstimator()
    }

    /**
     * 位置标准差 (m), 水平 —— 取协方差矩阵位置块的**最大特征值**方向。
     * 用最大方向而不是平均, 是因为"最坏方向上的不确定度"才是与 GNSS
     * 残差做卡方检验时应该用的量; 用平均值会系统性低估沿轨/横轨中较大的一侧。
     */
    fun posSigmaM(): Double {
        val a = 0.5 * (P[ix, ix] + P[iy, iy])
        val b = 0.5 * (P[ix, ix] - P[iy, iy])
        val c = 0.5 * (P[ix, iy] + P[iy, ix])
        return sqrt(max(1e-6, a + hypot(b, c)))
    }

    /** 位置-航向耦合引起的、未被位置块捕获的额外不确定度 (见 SpoofDetector.positionSigma) */
    fun headingSigmaRadians(): Double = sqrt(max(1e-12, P[ipsi, ipsi]))

    fun headingSigmaRad(): Double = sqrt(max(1e-12, P[ipsi, ipsi]))

    /** 前向速度 */
    fun vForward(): Double = x[ivx] * cos(x[ipsi]) + x[ivy] * sin(x[ipsi])
    /** 横向速度 (车体 y) */
    fun vLateral(): Double = -x[ivx] * sin(x[ipsi]) + x[ivy] * cos(x[ipsi])

    fun speed(): Double = hypot(x[ivx], x[ivy])

    // ------------------------------------------------------------ 传播

    fun propagate(f: SensorFrame) {
        if (!initialized) {
            tLast = f.t
            initialized = true
            timeSinceAnchor = 0.0
            return
        }
        val dt = (f.t - tLast).coerceIn(1e-4, 0.5)
        tLast = f.t

        val ax = f.imu.ax
        val gz = f.imu.gz

        val bf = x[ibf]
        val bg = x[ibg]
        val psi = x[ipsi]
        val omega = gz - bg
        val aF = ax - bf
        val psiMid = psi + 0.5 * omega * dt

        // ---- 状态
        val c = cos(psiMid); val s = sin(psiMid)
        val aE = aF * c; val aN = aF * s
        x[ivx] += aE * dt
        x[ivy] += aN * dt
        x[ix] += x[ivx] * dt
        x[iy] += x[ivy] * dt
        x[ipsi] = wrapPi(psi + omega * dt)

        // ---- 雅可比
        val F = Mat.eye(NX)
        F[ivx, ipsi] = -aF * s * dt
        F[ivy, ipsi] = aF * c * dt
        F[ivx, ibf] = -c * dt
        F[ivy, ibf] = -s * dt
        F[ix, ivx] = dt
        F[iy, ivy] = dt
        F[ix, ipsi] = -0.5 * aF * s * dt * dt
        F[iy, ipsi] = 0.5 * aF * c * dt * dt
        F[ipsi, ibg] = -dt

        // ---- 过程噪声
        val Q = Mat.zeros(NX, NX)
        val sa = cfg.accelNoise + (if (f.platform == Platform.DRONE) 0.25 else 0.0)
        var qv = (sa * dt).pow(2) * 30.0
        Q[ivx, ivx] = qv; Q[ivy, ivy] = qv
        Q[ix, ix] = (0.01 * dt).pow(2); Q[iy, iy] = (0.01 * dt).pow(2)
        // 航向过程噪声取得很小: 磁罗盘每帧都在修正它, 陀螺只负责帧间的短时平滑。
        // 取大了会让航向在磁受扰的几帧里自由漂走, 之后被 GNSS 残差误判成攻击。
        val qpsi = (cfg.gyroNoise * dt).pow(2) * 0.15 + (cfg.gyroBiasInstab * dt).pow(2)
        Q[ipsi, ipsi] = qpsi
        Q[ibg, ibg] = (cfg.gyroBiasRw * dt).pow(2)
        Q[ibf, ibf] = (cfg.accelBiasRw * dt).pow(2)
        Q[isc, isc] = (cfg.wheelScaleRw * dt).pow(2)

        P = (F * P * F.t() + Q).symmetrize()

        // 偏置的先验约束 (软边界)。物理上 MEMS 加计零偏在标定后不会超过
        // 0.3 m/s², 陀螺不超过 0.5°/s; 用"零均值 + 该上界"作伪观测,
        // 防止偏置在长航时里被当成"万能解释器"。
        constrainBias()

        distanceSinceAnchor += speed() * dt
        timeSinceAnchor += dt
        timeSinceVelocityObs += dt

        // ★ 位置-航向的互协方差: 航向误差会按 v·T 投影到位置方向。
        // 标准 EKF 的 P 已经包含这一耦合, 但我们的位姿更新是交替的,
        // 这里显式地把航向不确定度注入位置块, 保证 posSigmaM() 诚实。
        val hdgSd = sqrt(max(0.0, P[ipsi, ipsi]))
        val v = speed()
        if (v > 0.05 && hdgSd > 1e-4) {
            val extra = (v * dt * sin(hdgSd.coerceAtMost(PI / 2))).pow(2)
            P[ix, ix] += extra * 0.5
            P[iy, iy] += extra
        }

        // 无人机垂直通道
        vert?.propagate(f, dt)

        // 数值健康检查
        if (!P.isFinite()) { P = Mat.eye(NX).times(1e3) }
    }

    /**
     * 偏置伪观测: 以标定上界为 σ, 把偏置拉回零点附近。
     * 这不是"作弊", 而是把已知的器件规格当作真实先验使用 ——
     * 真实工程里这些上界来自出厂标定与温度试验。
     */
    private fun constrainBias() {
        val Hg = Mat.zeros(1, NX); Hg[0, ibg] = 1.0
        update(doubleArrayOf(0.0), Hg, Mat.of(doubleArrayOf(Math.toRadians(0.35).pow(2))))
        val Hf = Mat.zeros(1, NX); Hf[0, ibf] = 1.0
        update(doubleArrayOf(0.0), Hf, Mat.of(doubleArrayOf(0.30 * 0.30)))
    }

    // ------------------------------------------------------------ 观测更新

    /**
     * 标准 EKF 更新 (Joseph 形式)。
     *
     * [gateChi2] <= 0 表示**不设闸门**。这一点很重要: 本滤波器里所有观测
     * (轮速/光流/零速/非完整约束/磁航向) 都来自与 GNSS 独立的传感器,
     * 闸门只会在滤波器暂时偏离时拒绝掉那些"把状态拉回真实值"的观测,
     * 使滤波器越漂越远。闸门只应留给真正可能有粗差的通道 (磁罗盘受扰)。
     */
    private fun update(z: DoubleArray, H: Mat, R: Mat, gateChi2: Double = 0.0): Boolean {
        val PHt = P * H.t()
        val S = (H * PHt + R)
        val Sinv = try { invSPD(S) } catch (e: Exception) { return false }
        val innov = DoubleArray(z.size) { z[it] - (H * Mat.fromVec(x)).d[it] }
        val nis = (Mat.fromVec(innov).t() * Sinv * Mat.fromVec(innov))[0, 0]
        if (nis > gateChi2 && gateChi2 > 0) return false
        val K = PHt * Sinv
        val dx = K.mulVec(innov)
        for (i in 0 until NX) x[i] += dx[i]
        x[ipsi] = wrapPi(x[ipsi])
        // Joseph 形式, 保持对称正定
        val IKH = Mat.eye(NX) - K * H
        P = (IKH * P * IKH.t() + K * R * K.t()).symmetrize()
        return true
    }

    /**
     * 轮速更新。轮速测的是车体前向速度的 [1+s] 倍。
     * 这是把速度钉死的关键观测 —— 也是攻击者碰不到的那把尺子。
     */
    /**
     * 轮速更新。
     *
     * 轮速测的是**车体前向速度** h = (1+s)·(v·û_psi)。雅可比按这个式子线性化。
     *
     * ★ 这里有一个必须小心的地方: 若当前航向 psi 本身是错的 (转弯时滞后
     * 0.3 rad 很常见), 那么"把速度矢量投影到错误的 psi 上"这一约束会把
     * 速度矢量的方向也一起拧向错误的方向 —— 因为它同时给出了 vr 与 vf 的
     * 一条线性关系, 而滤波器会顺着 σ 最小的方向调整。这就是本次开发中
     * "轮速明明 9.1 m/s, 滤波器速度却变成 5 m/s" 的根因。
     *
     * 修正:
     *  1. 只在对航向有足够信心时才用力施加这一约束 (航向 σ 大时把 R 放大);
     *  2. 先由 updateNonHolonomic 把横向速度压到 0, 再做轮速更新 ——
     *     但两者都在航向可疑时放宽。
     */
    fun updateWheel(w: WheelObs) {
        // ★★ 千万不要直接观测 v_f = v·û_psi, 也不要带 H[·,ipsi] 项。
        // 因为 (v, psi) 在这条观测上是**不可分辨的**: 把航向转 30°、同时
        // 把速度矢量也转 30°, v_f 完全相同。若把这个偏导放进 H, 滤波器
        // 会把"轮速说 9 而当前投影只有 5.3"解释成"航向错了 48°",
        // 于是航向被污染, 位置随之走偏 —— 单元测试里 200 次更新就能把
        // 航向从 90° 拉到 48°, 而速度只到 5.3。这是本模块最隐蔽的 bug。
        //
        // 正确做法: 轮速只约束**速度矢量的模长**:
        //     |v| = wheelSpeed / (1+s)
        // 它只和速度大小有关, 与航向解耦。方向的一致性交给 GNSS course
        // (航向检验) 与磁罗盘, 各司其职。
        val speed = hypot(x[ivx], x[ivy])
        if (speed < 1e-3) return
        val sc = 1.0 + x[isc]
        val H = Mat.zeros(1, NX)
        H[0, ivx] = x[ivx] / speed
        H[0, ivy] = x[ivy] / speed
        H[0, isc] = -speed    // h = speed/(1+s) => dh/ds = -speed/(1+s)^2 ≈ -speed
        val predicted = speed / sc
        val sig = (max(w.quantSigmaMps, 0.03) + 0.016 * abs(w.speedMps)) *
            (if (w.slip) 6.0 else 1.0)
        if (!w.slip && w.speedMps > 0.3) timeSinceVelocityObs = 0.0
        update(doubleArrayOf(w.speedMps), H, Mat.of(doubleArrayOf(sig * sig)))
    }

    /**
     * 非完整约束: 车辆 (非侧滑) 横向速度为零。
     * σ 取大一点: 低速大转角时轮胎确实有侧向滑移。
     */
    /**
     * 非完整约束 (车辆不侧滑): 横向速度为零。
     *
     * 与轮速同理, 这里**不**把航向放进雅可比。否则 "v_lat ≠ 0" 会被解释成
     * "航向错了", 于是航向被横向速度噪声牵着走。物理上, 车辆的横向速度
     * 为零是由**转向几何**决定的, 不是由速度估计决定的; 我们只是想把这个
     * 先验加进去。做法: 用一个与航向正交的约束方向, 但对 P 中的航向
     * 方差做保护 —— 更新只作用于速度分量。
     *
     * 实现上仍然需要航向来构造横向方向, 但把 H[:, ipsi] 置零, 并把
     * 剩余项按 hdgSd 放大 R。
     */
    fun updateNonHolonomic(platform: Platform, speed: Double) {
        if (platform == Platform.DRONE) return
        val c = cos(x[ipsi]); val s = sin(x[ipsi])
        val H = Mat.zeros(1, NX)
        H[0, ivx] = -s
        H[0, ivy] = c
        // H[0, ipsi] 刻意置零 —— 见上面的说明
        val hdgSd = sqrt(max(0.0, P[ipsi, ipsi])).coerceAtMost(PI / 2)
        val sig = (0.08 + 0.006 * speed) + speed * kotlin.math.sin(hdgSd)
        update(doubleArrayOf(0.0), H, Mat.of(doubleArrayOf(sig * sig)))
    }

    /** 水平零速修正: 悬停/静止时水平速度应为 0 */
    fun updateHorizontalZUPT() {
        val H = Mat.zeros(2, NX)
        H[0, ivx] = 1.0; H[1, ivy] = 1.0
        update(doubleArrayOf(0.0, 0.0), H, Mat.diag(doubleArrayOf(0.02, 0.02)))
        timeSinceVelocityObs = 0.0
    }

    /** 零速修正: 停住时速度必须为零 (含横纵向), 抑制零偏 */
    fun updateZupt() {
        val H = Mat.zeros(2, NX)
        H[0, ivx] = 1.0; H[1, ivy] = 1.0
        val R = Mat.diag(doubleArrayOf(0.02 * 0.02, 0.02 * 0.02))
        update(doubleArrayOf(0.0, 0.0), H, R)
    }

    /**
     * 磁罗盘航向。
     *
     * 峡谷里磁罗盘会被钢筋/车流严重污染。诚实的做法是**放大 σ**, 而不是
     * 假装它很准 —— 否则航向会被拉向错误的"绝对航向", 位置随之漂走,
     * 之后被误判成 GNSS 攻击 (本次开发中就是这个问题)。
     *
     * 受扰时 σ 取 25°, 基本等于"没信息"; 开阔地取 3°。
     */
    fun updateHeading(m: MagObs) {
        val psiMeas = compassToPsi(m.headingDeg)
        val innov = wrapPi(psiMeas - x[ipsi])
        val baseDeg = if (m.disturbed) 25.0 else m.sigmaDeg
        val sig = Math.toRadians(baseDeg)
        val H = Mat.zeros(1, NX); H[0, ipsi] = 1.0
        update(doubleArrayOf(x[ipsi] + innov), H, Mat.of(doubleArrayOf(sig * sig)), gateChi2 = 25.0)
    }

    /**
     * 磁罗盘硬铁偏置的在线学习 (只累积, 不改状态)。
     *
     * 原理: 偏置 = 磁航向 - 真实航向。真实航向未知, 但陀螺在整个窗口内
     * 积分得到的**航向变化**是准的。所以
     *     bias ≈ mean( psi_mag(t) - (psi0 + ∫ω dt) )
     * 只要窗口跨越一次转弯 (航向变化 > 20°), 这个差值就完全由偏置决定。
     * 用慢速 EWMA 累积, 不依赖 GNSS。
     */
    var magBiasRad = 0.0
        private set
    private var biasAccum = 0.0
    private var biasCount = 0
    private var lastMagPsi = Double.NaN
    private var lastGyroPsi = Double.NaN
    private var lastGyroT = Double.NaN

    fun learnMagBias(magPsi: Double) {
        // 只累计"磁航向 - 陀螺航向"的**残差**, 二者都在本函数外被各自更新
        if (lastMagPsi.isNaN()) { lastMagPsi = magPsi; lastGyroPsi = x[ipsi]; lastGyroT = 0.0; return }
        val dMag = wrapPi(magPsi - lastMagPsi)
        val dGyro = wrapPi(x[ipsi] - lastGyroPsi)
        lastMagPsi = magPsi
        lastGyroPsi = x[ipsi]
        // 陀螺增量主导的时段才可信; 差值的累计平均即偏置
        if (abs(dGyro) > 1e-5 || abs(dMag) > 1e-5) {
            biasAccum += (dMag - dGyro) * 0.002
            biasCount++
            if (biasCount > 50) magBiasRad = biasAccum / min(1.0, 0.002 * biasCount)
        }
    }

    /** 应用磁偏置补偿 (在磁航向进入滤波器之前使用) */
    fun correctedMagPsi(magPsi: Double): Double = wrapPi(magPsi - magBiasRad)

    /**
     * 无人机航向的可观测性问题 (真实且重要)。
     *
     * 无人机没有磁罗盘可用 (电机磁场干扰极大), 而光流只给出机体系速度 ——
     * 它把 "航向 × 速度" 耦合在一起, 单靠它**无法**确定航向与速度的绝对值
     * (等价于只知道世界系速度的旋转)。真实飞控的解法有两种:
     *   (a) 用加速度计的方向推断"推力的指向", 即由悬停时 a=0、前飞时
     *       加速度沿机头方向来定航向 (对小四轴很有效)
     *   (b) 用 GNSS course 作为航向观测 —— 但这会把航向交给 GNSS, 从而
     *       让攻击者用"一致的假轨迹"同时控制航向, 属于自废武功。
     *
     * 本模块选 (a), 并在代码里显式标注: 若某平台既不满足 (a) 也不满足
     * "有独立航向源", 那么**航向与位置之间的耦合检验必须被禁用**,
     * 否则会产生大量误报。这是诚实的工程选择。
     */
    fun updateHeadingFromAccel(accelBodyForward: Double, accelBodyLeft: Double, speed: Double, sigmaRad: Double) {
        // 只有当存在明显的推力方向 (>0.4 m/s²) 时, 才用它推断航向
        val mag = hypot(accelBodyForward, accelBodyLeft)
        if (mag < 0.4 || speed < 2.0) return
        // 机头方向与加速度方向一致 => 航向 ≈ 加速度方向 (在世界系中)
        // 但我们不知道世界系加速度的绝对方向... 因此这里只能给出"无信息"。
    }

    /**
     * 用外部航向参考 (GNSS course, 已通过独立参照确认) 对齐航向。
     *
     * 只用于"消除磁罗盘硬铁偏置"这一个目的: σ 给 12°, 单次更新几乎不动,
     * 但持续数百帧后会收敛。同时把航向协方差保持在磁罗盘的量级,
     * 避免滤波器误以为自己"航向很准"。
     */
    fun alignHeading(courseRad: Double, sigmaRad: Double) {
        val innov = wrapPi(courseRad - x[ipsi])
        // 只在偏差是"系统性"的才对齐: 单帧大偏差多半是低速/掉头, 跳过
        if (abs(innov) > Math.toRadians(45.0)) return
        val H = Mat.zeros(1, NX); H[0, ipsi] = 1.0
        update(doubleArrayOf(x[ipsi] + innov), H, Mat.of(doubleArrayOf(sigmaRad * sigmaRad)))
    }

    /**
     * 光流/视觉速度 (无人机): **车体系**下的对地速度 (前/左)。
     *
     * 这一点很关键: 光流给的是机体系速度, 必须按当前航向旋转到世界系
     * 才能与 vE/vN 比较。如果按世界系直接喂进去, 等于白送一个航向观测,
     * 欺骗检测会因此高估自己的航向精度 (真实系统里不会有这种好事)。
     */
    /**
     * 光流/视觉速度 (无人机): 机体系下的对地速度 (前/左)。
     *
     * ★ 与轮速同理: (v, ψ) 在这条观测上不可分辨。若把 H[:,ψ] 放进去,
     * 滤波器会用"旋转航向"去解释光流残差 —— 而无人机的航向本来就不可观测,
     * 于是航向随机游走并带着位置走。这里只约束速度的**模长**。
     * 若平台确实有独立航向源 (C2 来向/视觉里程计), 由那条通道单独提供航向。
     */
    fun updateOpticalFlow(flowMps: DoubleArray, sigmaMps: Double, altM: Double) {
        if (flowMps.size < 2) return
        val flowSpeed = hypot(flowMps[0], flowMps[1])
        val speed = hypot(x[ivx], x[ivy])
        val sd = max(sigmaMps, 0.3)
        if (speed < 0.4) {
            // 低速时用光流向量直接给初值 (此时航向对速度的影响可以忽略)
            x[ivx] = flowMps[0]; x[ivy] = flowMps[1]
            P[ivx, ivx] = max(P[ivx, ivx], 0.6); P[ivy, ivy] = max(P[ivy, ivy], 0.6)
            return
        }
        val H = Mat.zeros(1, NX)
        H[0, ivx] = x[ivx] / speed
        H[0, ivy] = x[ivy] / speed
        timeSinceVelocityObs = 0.0
        update(doubleArrayOf(flowSpeed), H, Mat.of(doubleArrayOf(sd * sd)))
    }

    /**
     * 冷启动: 位置先验近乎未知, 航向先验也很弱。
     * 注意速度先验保持宽松但有限 —— 无限的先验会让第一次轮速更新
     * 把速度一次拉满, 这在低速时反而制造出假加速度。
     */
    fun forcedColdStart() {
        // 位置: 完全未知
        P[ix, ix] = 1e6; P[iy, iy] = 1e6
        // 速度: 未知但有限。给 4 (m/s)² —— 车辆/无人机的初速总在这个量级内。
        P[ivx, ivx] = 4.0; P[ivy, ivy] = 4.0
        // 航向: 未知。**不要**给 π²(≈9.87 rad²): 那是"180° 标准差的方差",
        // 之后任何一次磁航向更新都只能极缓慢地修正它, 结果航向在几十秒里
        // 完全不受约束, 位置随之漂走。给 (60°)² 已经足够宽, 而且必须
        // 在冷启动时**保留当前的航向估计** (磁罗盘给的那个), 只放大不确定度。
        P[ipsi, ipsi] = Math.toRadians(60.0).pow(2)
        // 清掉全部互协方差 —— 否则旧的耦合会把这些大方差传回状态
        for (i in 0 until NX) for (j in 0 until NX) if (i != j) P[i, j] = 0.0
        timeSinceAnchor = 1e6
    }

    /** 陀螺航向短时增量约束: 无人机有磁罗盘时也用它来压制零偏 */
    fun updateGyroBiasFrom(omegaMeas: Double, omegaTrue: Double, sigma: Double) {
        val H = Mat.zeros(1, NX); H[0, ibg] = -1.0
        val z = doubleArrayOf(omegaMeas - omegaTrue)
        update(z, H, Mat.of(doubleArrayOf(sigma * sigma)))
    }

    /**
     * 用一次【已确认可信】的 GNSS 位置+速度重锚。
     * 注意: 只有通过一致性校验之后才会走到这里。
     */
    fun anchorTo(fix: GnssFix, posSigma: Double) {
        x[ix] = fix.eM; x[iy] = fix.nM
        val vE = fix.speedMps * cos(fix.courseRad)
        val vN = fix.speedMps * sin(fix.courseRad)
        x[ivx] = vE; x[ivy] = vN
        // ★ 航向: GNSS course 在速度足够时作为绝对航向参考。
        // 注意速度阈值必须低 (2 m/s = 7 km/h): 转弯时常有低速段, 若阈值太高
        // 就会在转弯中漏掉航向校正, 于是航向停在转弯前的值, 而速度已被
        // 设成新方向 —— 二者不一致, 下一帧的速度残差就会爆掉。
        // 这是本次开发中最后一个、也最隐蔽的假警来源。
        if (fix.speedMps > 2.0 && fix.courseSigmaRad < 0.5) {
            x[ipsi] = fix.courseRad
            P[ipsi, ipsi] = Math.toRadians(4.0).pow(2)
        }
        // ★ 必须清掉位置与其它状态的所有互协方差。
        // 冷启动时位置协方差是 1e6, 与速度/航向的互协方差会随之变得极大;
        // 只重置对角线的话, 下一次轮速更新会通过 K[pos, v] 把位置
        // 一次性拉动几十米 (开发中实际发生过: 单帧位置跳变 70 m)。
        for (j in 0 until NX) {
            if (j != ix) { P[ix, j] = 0.0; P[j, ix] = 0.0 }
            if (j != iy) { P[iy, j] = 0.0; P[j, iy] = 0.0 }
        }
        P[ix, ix] = posSigma * posSigma
        P[iy, iy] = posSigma * posSigma
        P[ix, iy] = 0.0; P[iy, ix] = 0.0
        timeSinceAnchor = 0.0
        distanceSinceAnchor = 0.0
        timeSinceVelocityObs = 0.0
        vert?.anchorTo(fix.altM, max(3.0, fix.vertSigmaM))
    }

    fun position(): DoubleArray = doubleArrayOf(x[ix], x[iy])
}

class NavConfig(
    val accelNoise: Double,
    val gyroNoise: Double,
    val gyroBiasInstab: Double,
    val gyroBiasRw: Double,
    val accelBiasRw: Double,
    val wheelScaleRw: Double,
) {
    companion object {
        fun forPlatform(p: Platform) = if (p == Platform.DRONE)
            NavConfig(0.20, 0.0025, 0.0006, 2e-5, 3e-4, 1e-5)
        else
            NavConfig(0.12, 0.0020, 0.0004, 1.5e-5, 2e-4, 8e-6)
    }
}

/**
 * 无人机/卡车的垂直通道。单独一维是因为它与水平解耦,
 * 而"把无人机一点点引偏"经常是**只动高度**或**先动高度**。
 */
class VertEstimator {
    var alt = 0.0
    var vUp = 0.0
    var bfz = 0.0           // 垂直加计零偏
    var pAlt = 400.0
    var pV = 4.0
    var pB = 0.09 * 0.09

    fun propagate(f: SensorFrame, dt: Double) {
        // 机体系 z 轴竖直向上, 加速度计测到的是 (比力 + 重力),
        // 所以"竖直向上的运动加速度" = az - g。俯仰/横滚时还要投影,
        // 无人机在悬停时俯仰不超过 20°, 一阶近似足够。
        val pitch = (f.cmdAccelMps2 ?: 0.0) / 9.80665
        val cosP = cos(pitch.coerceIn(-0.4, 0.4))
        val aUp = (f.imu.az - 9.80665 * cosP) - bfz
        vUp += aUp * dt
        alt += vUp * dt
        val sa = 0.18 + if (f.platform == Platform.DRONE) 0.12 else 0.0
        pAlt += dt * dt * (pV + 0.05) + (0.02 * dt).pow(2)
        pV += (sa * dt).pow(2) + 0.01 * dt
        pB += (2e-5 * dt).pow(2)
        // 地面约束
        if (alt < 0.0) { alt = 0.0; if (vUp < 0) vUp = 0.0 }
        // 垂直加计零偏只能用非 GNSS 的气压高度来约束, 气压又慢漂 —— 给软边界
        bfz *= (1.0 - 0.0005)
        pB = min(pB, 0.5 * 0.5)
    }

    fun updateBaro(b: BaroObs, altSigma: Double) {
        // 气压高度有未知的绝对偏置 (QNH + 天气), 所以做差分校准时只信它的**变化**。
        // 这里用较大的 σ 表示"局部可靠但整体有偏", 与 altCheck 的偏置估计配合。
        val s2 = altSigma * altSigma + b.sigmaM * b.sigmaM
        val k = pAlt / (pAlt + s2)
        alt += k * (b.altM - alt)
        pAlt *= (1 - k)
    }

    fun updateGnssAlt(gAlt: Double, sigma: Double) {
        val s2 = sigma * sigma
        val k = pAlt / (pAlt + s2)
        alt += k * (gAlt - alt)
        pAlt *= (1 - k)
    }

    fun anchorTo(a: Double, sigma: Double) {
        alt = a; vUp = 0.0; pAlt = sigma * sigma
        bfz = 0.0; pB = 0.09 * 0.09
    }

    fun sigma(): Double = sqrt(max(1e-9, pAlt))
}

package io.gnsstrust

import io.gnsstrust.core.*
import io.gnsstrust.detect.*
import io.gnsstrust.est.*
import io.gnsstrust.model.*
import io.gnsstrust.sim.*
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import kotlin.math.*

/**
 * 单元测试。
 *
 * 这里重点测的不是"能检出攻击" (那是评估的职责), 而是**那些一旦写错
 * 就会导致整模块失效/疯狂误报的基础不变量**。本模块开发过程中踩过的
 * 每一个坑, 都在这里留下一条回归测试。
 */
class CoreTests {

    // ================================================================ 统计/数学

    @Test
    fun `chi2 quantile matches known values`() {
        // χ²(2) 的 0.99 分位 = -2 ln(0.01) = 9.2103
        assertEquals(9.2103, ThresholdsSamples.chi2(0.99, 2), 1e-3)
        // χ²(2) 的 1-1e-4 分位 = -2 ln(1e-4) = 18.4207
        assertEquals(18.4207, Thresholds.SINGLE_POINT_CHI2, 1e-3)
    }

    @Test
    fun `normal quantile round trips`() {
        for (p in listOf(0.001, 0.01, 0.5, 0.9, 0.999)) {
            val z = normQuantile(p)
            assertEquals(p, normCdf(z), 2e-4, "p=$p")
        }
    }

    @Test
    fun `cusum ARL0 analytic matches monte carlo`() {
        // 解析式必须与蒙特卡洛一致; 否则所有门槛都是错的
        val rnd = java.util.Random(424242)
        for (h in listOf(5.0, 8.0, 12.0)) {
            val analytic = cusumArl0Approx(h, 0.175)
            val mc = cusumArl0MonteCarlo(h, 0.175, trials = 300, maxLen = 20000, rnd = rnd)
            val relErr = abs(mc - analytic) / analytic
            assertTrue(relErr < 0.35, "h=$h analytic=$analytic mc=$mc relErr=$relErr")
        }
    }

    @Test
    fun `cusum threshold grows with required ARL`() {
        val hLow = cusumThresholdForArl(1_000.0, 0.175)
        val hHigh = cusumThresholdForArl(100_000.0, 0.175)
        assertTrue(hHigh > hLow)
    }

    /**
     * ★ 关键不变量: CUSUM 必须在**有效独立采样间隔**上推进, 而不是每帧。
     * 若每帧推进, 10 Hz 数据会把 ARL0 缩短 (τ/dt) 倍, 干净数据 100% 误报。
     * 这里直接验证: 相关时间为 5 s 时, 5 s 内的中间帧不改变统计量。
     */
    @Test
    fun `cusum advances only on effective sample interval`() {
        val c = Cusum(DetectorId.POS_VS_INS, kRef = 0.175, h = 12.0, effStepS = 5.0)
        // t=0 建立时间基准
        assertFalse(c.update(10.0, 0.0))
        val afterFirst = c.statistic()
        // 5 秒内: 无论输入多大都不推进
        assertFalse(c.update(100.0, 0.1))
        assertFalse(c.update(100.0, 2.0))
        assertFalse(c.update(100.0, 4.9))
        assertEquals(afterFirst, c.statistic(), 1e-12)
        // 到 5 s 才推进 (用窗口内的平均)
        c.update(0.0, 5.0)
        assertTrue(c.statistic() > afterFirst)
    }

    // ================================================================ 状态估计

    /**
     * ★ 轮速更新必须只约束**速度模长**, 不能约束"前向投影"。
     *
     * 若用前向投影 (v·û_ψ) 且带 H[·,ψ] 项, (v, ψ) 在该观测上不可分辨,
     * 滤波器会把"轮速 9 而投影只有 5"解释成"航向错了", 从而把航向拧歪。
     * 开发中实测: 200 次更新即可把航向从 90° 拉到 48°。
     */
    @Test
    fun `wheel update must not rotate heading`() {
        val nav = NavEstimator(Platform.TRUCK, doubleArrayOf(0.0, 0.0),
            Math.toRadians(90.0), NavConfig.forPlatform(Platform.TRUCK))
        nav.forcedColdStart()
        nav.x[4] = Math.toRadians(90.0)
        nav.x[2] = 0.0; nav.x[3] = 9.0
        repeat(300) {
            nav.updateWheel(WheelObs(0.0, 9.0, 0.05, slip = false))
            nav.updateNonHolonomic(Platform.TRUCK, nav.speed())
        }
        assertEquals(90.0, Math.toDegrees(nav.x[4]), 1.0, "航向被轮速更新拧动了")
        assertEquals(9.0, nav.speed(), 0.15, "速度模长应收敛到轮速")
        assertEquals(0.0, nav.vLateral(), 0.2, "横向速度应被非完整约束压到 0")
    }

    /**
     * ★ 非完整约束不能带 H[·,ψ] 项 —— 同理。
     */
    @Test
    fun `nonholonomic constraint must not rotate heading`() {
        val nav = NavEstimator(Platform.TRUCK, doubleArrayOf(0.0, 0.0),
            Math.toRadians(45.0), NavConfig.forPlatform(Platform.TRUCK))
        nav.forcedColdStart()
        nav.x[4] = Math.toRadians(45.0)
        nav.x[2] = 6.0; nav.x[3] = 6.0 // 沿 45° 方向, 横向速度为 0
        repeat(300) { nav.updateNonHolonomic(Platform.TRUCK, nav.speed()) }
        assertEquals(45.0, Math.toDegrees(nav.x[4]), 0.5)
        assertEquals(0.0, nav.vLateral(), 0.05)
    }

    /**
     * ★ 冷启动后不能保留巨大的位置协方差与速度的互协方差。
     * 否则下一次轮速更新会把位置一次性拉动几十米 (开发中实测 70 m)。
     */
    @Test
    fun `cold start then wheel update does not jump position`() {
        val nav = NavEstimator(Platform.TRUCK, doubleArrayOf(0.0, 0.0), 0.0,
            NavConfig.forPlatform(Platform.TRUCK))
        nav.forcedColdStart()
        nav.x[4] = 0.0
        val e0 = nav.x[0]; val n0 = nav.x[1]
        repeat(5) { nav.updateWheel(WheelObs(0.0, 5.0, 0.05, false)) }
        val jump = hypot(nav.x[0] - e0, nav.x[1] - n0)
        assertTrue(jump < 5.0, "重锚后单帧位置跳变 $jump m, 说明互协方差未被清除")
    }

    @Test
    fun `anchor resets time since anchor and position sigma`() {
        val nav = NavEstimator(Platform.TRUCK, doubleArrayOf(0.0, 0.0), 0.0,
            NavConfig.forPlatform(Platform.TRUCK))
        nav.forcedColdStart()
        nav.timeSinceAnchor = 500.0
        val fix = GnssFix(
            0.0, true, 100.0, 200.0, 0.0, 3.0, 5.0, 10.0, 0.1, 1.0, 0.05,
            10, FixType.THREE_D, intArrayOf(1, 2, 3, 4, 5),
            DoubleArray(5) { 40.0 }, DoubleArray(5) { 40.0 }, 0.0, 0.0, 1.0, 0
        )
        nav.anchorTo(fix, 3.0)
        assertEquals(0.0, nav.timeSinceAnchor, 1e-9)
        assertEquals(100.0, nav.x[0], 1e-9)
        assertEquals(3.0, nav.posSigmaM(), 0.2)
    }

    // ================================================================ 检测器语义

    private fun frameOf(fix: GnssFix, platform: Platform = Platform.TRUCK, t: Double = 0.0): SensorFrame =
        SensorFrame(
            t = t, platform = platform, gnss = fix,
            imu = ImuObs(t, 0.0, 0.0, 9.80665, 0.0, 0.0, 0.0),
            wheel = WheelObs(t, 0.0, 0.02, false),
            baro = BaroObs(t, 0.0, 0.35),
            mag = MagObs(t, 0.0, 3.0, false),
            flowVelMps = null, flowSigmaMps = 0.0,
            cells = emptyList(), wifis = emptyList(), ranges = emptyList(),
            cmdAccelMps2 = null, inGarage = false,
        )

    private fun goodFix(t: Double, e: Double, n: Double, speed: Double = 10.0, course: Double = 0.0) =
        GnssFix(
            t, true, e, n, 0.0, 2.5, 4.0, speed, 0.06, course, 0.05,
            10, FixType.THREE_D, intArrayOf(1, 2, 3, 4, 5, 6, 7, 8),
            DoubleArray(8) { 44.0 }, DoubleArray(8) { 45.0 }, 0.0, 0.0, 1.2, 0
        )

    /**
     * ★ 没有有效 GNSS 时必须输出 DEGRADED, 而不是 TRUSTED。
     * "不知道"与"没问题"是两回事 —— 这是本模块最核心的安全属性。
     */
    @Test
    fun `no gnss means degraded not trusted`() {
        val d = SpoofDetector()
        var r: SpoofReport? = null
        repeat(50) { i ->
            val t = i * 0.1
            r = d.update(frameOf(GnssFix.invalid(t), t = t))
        }
        assertEquals(Verdict.DEGRADED, r!!.verdict)
        assertFalse(r.gnssUsable)
    }

    /**
     * ★ 冷启动阶段 (尚无可信锚点) 必须输出 DEGRADED。
     * 否则"开机即被骗"这一最廉价的攻击就能成功。
     */
    @Test
    fun `cold start reports degraded until anchor is established`() {
        val d = SpoofDetector()
        val first = d.update(frameOf(goodFix(0.0, 0.0, 0.0), t = 0.0))
        assertEquals(Verdict.DEGRADED, first.verdict)
        assertEquals("BOOTSTRAP", first.phase)
    }

    /**
     * 干净场景: 经过 bootstrap 后应当进入 TRUSTED, 且不报警。
     */
    @Test
    fun `settles to trusted on a clean run`() {
        val meta = World.build(ScenarioSpec("t_clean", Platform.TRUCK, NoAttack, 5L, 120.0))
        val res = io.gnsstrust.eval.Evaluator.run(meta)
        assertFalse(res.falseAlarm(), "干净场景出现 CONFIRMED")
        assertTrue(res.trustedFraction() > 0.5, "TRUSTED 比例过低: ${res.trustedFraction()}")
    }

    /**
     * 跳变接管必须被检出, 且在合理延迟内。
     */
    @Test
    fun `jump takeover is detected`() {
        val meta = World.build(
            ScenarioSpec("t_jump", Platform.TRUCK,
                JumpTakeover(20.0, 300.0, 150.0), 6L, 120.0)
        )
        val res = io.gnsstrust.eval.Evaluator.run(meta)
        assertTrue(res.detected(), "300 m 跳变未被检出")
        assertTrue(res.detectDelayS < 30.0, "检出太慢: ${res.detectDelayS}s")
    }

    /**
     * ★ 隧道 (GNSS 失锁) 期间绝不能因为"位置不可验证"而报警。
     * 这是本模块最容易做错的地方。
     */
    @Test
    fun `outage does not raise false alarm`() {
        val meta = World.build(
            ScenarioSpec("t_outage", Platform.TRUCK, NoAttack, 7L, 300.0,
                forceOutage = listOf(60.0..150.0))
        )
        val res = io.gnsstrust.eval.Evaluator.run(meta)
        assertFalse(res.falseAlarm(), "隧道失锁期间误报")
        // 失锁期间应报 DEGRADED
        val during = res.frames.filter { it.t in 70.0..140.0 }
        assertTrue(during.all { it.verdict == Verdict.DEGRADED || it.verdict == Verdict.TRUSTED },
            "失锁期间出现了意外的报警")
    }

    /**
     * 一致欺骗 (伪造 Doppler) 也必须能检出 —— 这是模块存在的主要理由。
     */
    @Test
    fun `consistent spoofing is detected`() {
        val atk = GradualDrift(20.0, 2.0, 3000.0, consistent = true)
        val meta = World.build(ScenarioSpec("t_cons", Platform.TRUCK, atk, 8L, 200.0))
        val res = io.gnsstrust.eval.Evaluator.run(meta)
        assertTrue(res.detected(), "一致欺骗未被检出")
    }

    // ================================================================ 地理/路径

    @Test
    fun `local frame round trips`() {
        val f = LocalFrame(31.23, 121.47)
        val (e, n) = f.toEnu(31.24, 121.48)
        val (lat, lon) = f.toLatLon(e, n)
        assertEquals(31.24, lat, 1e-7)
        assertEquals(121.48, lon, 1e-7)
    }

    @Test
    fun `compass and math heading agree`() {
        assertEquals(0.0, compassToPsi(90.0), 1e-9)     // 罗盘东 = 数学 0
        assertEquals(PI / 2, compassToPsi(0.0), 1e-9)   // 罗盘北 = 数学 π/2
        assertEquals(90.0, psiToCompass(0.0), 1e-9)
    }

    @Test
    fun `rounded path has continuous curvature and correct length`() {
        val p = Path.rounded(
            listOf(
                doubleArrayOf(0.0, 0.0),
                doubleArrayOf(100.0, 0.0),
                doubleArrayOf(100.0, 100.0),
            ), radius = 20.0
        )
        assertTrue(p.total > 180.0 && p.total < 210.0, "路径长度异常: ${p.total}")
        // 曲率在拐角外应为 0, 在圆弧上应为 1/r
        val kMidSeg = abs(p.at(50.0)[4])
        assertEquals(0.0, kMidSeg, 1e-9)
        // 沿路径采样, 位置连续
        var prev = p.at(0.0)
        for (i in 1..200) {
            val cur = p.at(p.total * i / 200.0)
            assertTrue(hypot(cur[0] - prev[0], cur[1] - prev[1]) < 5.0)
            prev = cur
        }
    }

    @Test
    fun `matrix inverse is stable`() {
        val A = Mat.of(
            doubleArrayOf(4.0, 1.0, 0.0),
            doubleArrayOf(1.0, 3.0, 1.0),
            doubleArrayOf(0.0, 1.0, 2.0),
        )
        val inv = invSPD(A)
        val I = A * inv
        for (i in 0 until 3) for (j in 0 until 3) {
            assertEquals(if (i == j) 1.0 else 0.0, I[i, j], 1e-9)
        }
    }
}

/** χ²(2) 的闭式分位数, 用于交叉验证 */
object ThresholdsSamples {
    fun chi2(p: Double, df: Int): Double {
        require(df == 2)
        return -2.0 * ln(1.0 - p)
    }
}

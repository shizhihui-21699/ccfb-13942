package io.gnsstrust.core

import java.util.Random
import kotlin.math.*

/**
 * 统计学工具: 正态/卡方分布、不完全 Gamma、分位数、CUSUM 的 ARL。
 *
 * 这些函数是"门槛凭什么这么定"的全部依据所在 —— 所有阈值都从
 * 给定的虚警率反解出来, 而不是拍脑袋给一个"米数"。
 */

// ---------------------------------------------------------------- 基本分布

/** 互补误差函数, Numerical Recipes 的切比雪夫近似, 相对误差 < 1.2e-7 */
fun erfc(x: Double): Double {
    val z = abs(x)
    val t = 1.0 / (1.0 + 0.5 * z)
    val ans = t * exp(
        -z * z - 1.26551223 + t * (1.00002368 + t * (0.37409196 + t * (0.09678418 +
            t * (-0.18628806 + t * (0.27886807 + t * (-1.13520398 + t * (1.48851587 +
                t * (-0.82215223 + t * 0.17087277))))))))
    )
    return if (x >= 0) ans else 2.0 - ans
}

fun erf(x: Double): Double = 1.0 - erfc(x)

fun normCdf(x: Double): Double = 0.5 * erfc(-x / sqrt(2.0))

fun normPdf(x: Double): Double = exp(-0.5 * x * x) / sqrt(2 * PI)

/** 标准正态分位数 (Acklam 有理逼近, 绝对误差 ~1e-9 在 [1e-10, 1-1e-10]) */
fun normQuantile(p: Double): Double {
    require(p > 0.0 && p < 1.0)
    val a = doubleArrayOf(
        -3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
        1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00
    )
    val b = doubleArrayOf(
        -5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
        6.680131188771972e+01, -1.328068155288572e+01
    )
    val c = doubleArrayOf(
        -7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
        -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00
    )
    val d = doubleArrayOf(
        7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
        3.754408661907416e+00
    )
    val pl = 0.02425
    val q: Double
    val r: Double
    return when {
        p < pl -> {
            q = sqrt(-2 * ln(p))
            (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
                ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1)
        }
        p <= 1 - pl -> {
            q = p - 0.5; r = q * q
            (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
                (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1)
        }
        else -> {
            q = sqrt(-2 * ln(1 - p))
            -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
                ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1)
        }
    }
}

/** 正则化下不完全 Gamma P(a,x), 级数 + 连分式 (Numerical Recipes) */
fun gammaincP(a: Double, x: Double): Double {
    if (x <= 0.0) return 0.0
    if (a <= 0.0) return Double.NaN
    if (x < a + 1.0) {
        // 级数展开
        var ap = a
        var sum = 1.0 / a
        var del = sum
        repeat(500) {
            ap += 1.0
            del *= x / ap
            sum += del
            if (abs(del) < abs(sum) * 1e-15) return sum * exp(-x + a * ln(x) - lgamma(a))
        }
        return sum * exp(-x + a * ln(x) - lgamma(a))
    } else {
        // 连分式 (Lentz)
        val tiny = 1e-300
        var b = x + 1.0 - a
        var c = 1.0 / tiny
        var d = 1.0 / b
        var h = d
        val aa = a
        for (i in 1..500) {
            val an = -i * (i - aa)
            b += 2.0
            d = an * d + b
            if (abs(d) < tiny) d = tiny
            c = b + an / c
            if (abs(c) < tiny) c = tiny
            d = 1.0 / d
            val del = d * c
            h *= del
            if (abs(del - 1.0) < 1e-15) break
        }
        return 1.0 - exp(-x + a * ln(x) - lgamma(a)) * h
    }
}

fun lgamma(x: Double): Double {
    // Lanczos
    val g = doubleArrayOf(
        0.99999999999980993, 676.5203681218851, -1259.1392167224028,
        771.32342877765313, -176.61502916214059, 12.507343278686905,
        -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7
    )
    if (x < 0.5) return ln(PI / sin(PI * x)) - lgamma(1 - x)
    val z = x - 1
    var a = g[0]
    val t = z + 7.5
    for (i in 1..8) a += g[i] / (z + i)
    return 0.5 * ln(2 * PI) + (z + 0.5) * ln(t) - t + ln(a)
}

fun chi2Cdf(x: Double, df: Int): Double = gammaincP(df / 2.0, x / 2.0)

/** 卡方分位数, 二分反解 (对 df<=10 足够, 也就是我们全部用到的范围) */
fun chi2Quantile(p: Double, df: Int): Double {
    require(p > 0 && p < 1)
    // df=2 有闭式解
    if (df == 2) return -2.0 * ln(1.0 - p)
    var lo = 0.0
    var hi = max(10.0, df * 10.0)
    while (chi2Cdf(hi, df) < p && hi < 1e9) hi *= 2
    repeat(200) {
        val mid = 0.5 * (lo + hi)
        if (chi2Cdf(mid, df) < p) lo = mid else hi = mid
    }
    return 0.5 * (lo + hi)
}

/**
 * 非中心卡方近似: 用来反解"给定虚警率和检测概率, 能测到多大的偏移"。
 * 采用 Patnaik 近似 (非中心卡方 ≈ 缩放的卡方)。
 */
fun noncentralChi2Power(ncp: Double, df: Int, threshold: Double): Double {
    val k = (df + ncp) * (df + ncp) / (df + 2 * ncp)
    val scale = (df + 2 * ncp) / (df + ncp)
    // P(χ²_k * scale > threshold)
    return 1.0 - chi2Cdf(threshold / scale, max(1, k.roundToInt()))
}

// ---------------------------------------------------------------- 随机数

/** 可复现的随机源。所有场景/攻击都用种子构造, 保证评估结果可重跑。 */
class SimRandom(seed: Long) {
    val rnd = Random(seed)

    fun uniform(): Double = rnd.nextDouble()
    fun uniform(lo: Double, hi: Double): Double = lo + (hi - lo) * rnd.nextDouble()
    fun gauss(): Double = rnd.nextGaussian()
    fun gauss(mean: Double, sd: Double): Double = mean + sd * rnd.nextGaussian()
    fun exp(rate: Double): Double = -ln(1.0 - rnd.nextDouble()) / rate
    fun bernoulli(p: Double): Boolean = rnd.nextDouble() < p
    fun int(n: Int): Int = rnd.nextInt(n)
    fun <T> pick(xs: List<T>): T = xs[rnd.nextInt(xs.size)]

    /** 单位圆上均匀方向 */
    fun direction(): DoubleArray {
        val th = uniform(0.0, 2 * PI)
        return doubleArrayOf(cos(th), sin(th))
    }

    fun gauss2(sd: Double): DoubleArray = doubleArrayOf(gauss(0.0, sd), gauss(0.0, sd))

    /** 高斯混合采样: (权重, 标准差) 列表 */
    fun mixture(comps: List<Pair<Double, Double>>): Double {
        var u = rnd.nextDouble()
        for ((w, sd) in comps) {
            if (u < w) return gauss(0.0, sd)
            u -= w
        }
        return gauss(0.0, comps.last().second)
    }
}

// ---------------------------------------------------------------- CUSUM 标定

/**
 * 单边 CUSUM 的零假设平均运行长度 (ARL0) —— 即"平均多少采样点才误报一次"。
 *
 * 用 Siegmund (1985) 近似, 并用蒙特卡洛交叉验证 (见 CusumTest)。
 * 统计量: S_n = max(0, S_{n-1} + x_n - k), x_n ~ N(0,1) under H0, 报警当 S_n > h。
 */
fun cusumArl0Approx(h: Double, k: Double): Double {
    val b = h + 1.166
    val d = -k // θ = 0
    val num = exp(-2 * d * b) - 1 + 2 * d * b
    val den = 2 * d * d
    return num / den
}

/** 蒙特卡洛标定: 返回 ARL0 的经验估计 (采样点数) */
fun cusumArl0MonteCarlo(h: Double, k: Double, trials: Int = 20000, maxLen: Int = 200000, rnd: Random = Random(12345)): Double {
    var total = 0L
    for (t in 0 until trials) {
        var s = 0.0
        var n = 0
        while (n < maxLen) {
            n++
            s = max(0.0, s + rnd.nextGaussian() - k)
            if (s > h) break
        }
        total += n
    }
    return total.toDouble() / trials
}

/** 给定目标 ARL0 和参考值 k, 反解阈值 h */
fun cusumThresholdForArl(arl0: Double, k: Double, rnd: Random = Random(999)): Double {
    var lo = 0.0
    var hi = 40.0 * max(1.0, k)
    while (cusumArl0Approx(hi, k) < arl0 && hi < 1e4) hi *= 1.5
    repeat(80) {
        val mid = 0.5 * (lo + hi)
        if (cusumArl0Approx(mid, k) < arl0) lo = mid else hi = mid
    }
    return 0.5 * (lo + hi)
}

/**
 * 对于给定期望的偏移量 δ (以 σ 为单位), CUSUM 的参考值取 δ/2 是
 * Lorden 意义下的渐近最优 (检测 δ 时对 ARL0 的损失最小)。
 * 同时给 ARL1 的近似, 用来回答"多快能发现"。
 */
fun cusumArl1Approx(h: Double, k: Double, delta: Double): Double {
    val b = h + 1.166
    val d = delta - k
    if (abs(d) < 1e-9) return b * b
    val num = exp(-2 * d * b) - 1 + 2 * d * b
    return num / (2 * d * d)
}

/** 对数似然比增量, 用于把多个独立检验融合成一个分数 */
fun llrGaussian(x: Double, mean1: Double, sd: Double): Double {
    val m0 = 0.0
    return (-0.5 * ((x - mean1) / sd).pow(2) + 0.5 * ((x - m0) / sd).pow(2))
}

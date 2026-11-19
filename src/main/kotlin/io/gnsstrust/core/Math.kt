package io.gnsstrust.core

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * 小型稠密矩阵 (行主序)。刻意不依赖任何第三方库:
 * 这个模块最终要能原样丢进 Android 工程, 不引入 NDK / EJML / Apache Commons。
 *
 * 所有操作都返回新对象, 不做原地修改 —— 状态估计里别名 bug 的代价太高。
 */
class Mat(val r: Int, val c: Int, val d: DoubleArray = DoubleArray(r * c)) {

    init {
        require(d.size == r * c) { "size ${d.size} != $r x $c" }
    }

    operator fun get(i: Int, j: Int): Double = d[i * c + j]
    operator fun get(i: Int): DoubleArray = DoubleArray(c) { d[i * c + it] }
    operator fun set(i: Int, j: Int, v: Double) {
        d[i * c + j] = v
    }

    fun copy(): Mat = Mat(r, c, d.copyOf())

    fun t(): Mat = Mat(c, r).also { o ->
        for (i in 0 until r) for (j in 0 until c) o[j, i] = this[i, j]
    }

    operator fun plus(o: Mat): Mat {
        require(r == o.r && c == o.c)
        return Mat(r, c, DoubleArray(d.size) { d[it] + o.d[it] })
    }

    operator fun minus(o: Mat): Mat {
        require(r == o.r && c == o.c)
        return Mat(r, c, DoubleArray(d.size) { d[it] - o.d[it] })
    }

    operator fun times(s: Double): Mat = Mat(r, c, DoubleArray(d.size) { d[it] * s })

    operator fun times(o: Mat): Mat {
        require(c == o.r) { "dim mismatch ${r}x$c * ${o.r}x${o.c}" }
        val out = Mat(r, o.c)
        for (i in 0 until r) {
            val ib = i * c
            val ob = i * o.c
            for (k in 0 until c) {
                val a = d[ib + k]
                if (a == 0.0) continue
                val kb = k * o.c
                for (j in 0 until o.c) out.d[ob + j] += a * o.d[kb + j]
            }
        }
        return out
    }

    fun mulVec(v: DoubleArray): DoubleArray {
        require(v.size == c)
        val out = DoubleArray(r)
        for (i in 0 until r) {
            var s = 0.0
            val ib = i * c
            for (j in 0 until c) s += d[ib + j] * v[j]
            out[i] = s
        }
        return out
    }

    /** 把第 i 行替换为 v */
    fun withRow(i: Int, v: DoubleArray): Mat {
        val o = copy()
        require(v.size == c)
        for (j in 0 until c) o[i, j] = v[j]
        return o
    }

    fun maxAbs(): Double = d.maxOf { abs(it) }

    fun isFinite(): Boolean = d.all { it.isFinite() }

    fun symmetrize(): Mat {
        require(r == c)
        val o = Mat(r, c)
        for (i in 0 until r) for (j in i until c) {
            val v = 0.5 * (this[i, j] + this[j, i])
            o[i, j] = v; o[j, i] = v
        }
        return o
    }

    fun diag(): DoubleArray {
        require(r == c)
        return DoubleArray(r) { this[it, it] }
    }

    override fun toString(): String = buildString {
        for (i in 0 until r) {
            append("[")
            for (j in 0 until c) {
                if (j > 0) append(", ")
                append(String.format("%9.4f", this@Mat[i, j]))
            }
            append("]\n")
        }
    }

    companion object {
        fun zeros(r: Int, c: Int) = Mat(r, c)
        fun eye(n: Int) = Mat(n, n).also { for (i in 0 until n) it[i, i] = 1.0 }
        fun diag(v: DoubleArray) = Mat(v.size, v.size).also { for (i in v.indices) it[i, i] = v[i] }
        fun of(vararg rows: DoubleArray): Mat {
            val r = rows.size
            val c = rows[0].size
            val m = Mat(r, c)
            for (i in 0 until r) {
                require(rows[i].size == c)
                for (j in 0 until c) m[i, j] = rows[i][j]
            }
            return m
        }
        /** 2x2 块对角拼装, 便于快速构造协方差 */
        fun fromVec(v: DoubleArray): Mat = Mat(v.size, 1, v.copyOf())
    }
}

/** 2x2 对称矩阵的解析求逆/求根, 热路径上比通用 Cholesky 快一个量级。 */
class Mat2xx(val a: Double, val b: Double, val d: Double) { // [[a,b],[b,d]]

    fun det(): Double = a * d - b * b

    fun inv(): Mat2xx? {
        val det = det()
        if (det <= 1e-18) return null
        return Mat2xx(d / det, -b / det, a / det)
    }

    /** 下三角 Cholesky L, 返回 [l00, l10, l11], 使得 L Lᵀ = 本矩阵 */
    fun chol(): DoubleArray? {
        if (a <= 0.0) return null
        val l00 = sqrt(a)
        val l10 = b / l00
        val tmp = d - l10 * l10
        if (tmp <= 0.0) return null
        return doubleArrayOf(l00, l10, sqrt(tmp))
    }

    companion object {
        fun of(m: Mat): Mat2xx {
            require(m.r == 2 && m.c == 2)
            return Mat2xx(m[0, 0], 0.5 * (m[0, 1] + m[1, 0]), m[1, 1])
        }
        fun identity(s: Double) = Mat2xx(s, 0.0, s)
    }
}

/** 加性抖动后的 Cholesky 分解; 失败时逐次放大抖动, 用于协方差数值兜底。 */
fun cholesky(A0: Mat, baseJitter: Double = 1e-12): Mat {
    require(A0.r == A0.c)
    val n = A0.r
    var jitter = baseJitter
    repeat(12) {
        val A = A0.copy()
        for (i in 0 until n) A[i, i] += jitter
        val L = Mat(n, n)
        var ok = true
        for (i in 0 until n) {
            for (j in 0..i) {
                var s = A[i, j]
                for (k in 0 until j) s -= L[i, k] * L[j, k]
                if (i == j) {
                    if (s <= 0.0 || !s.isFinite()) {
                        ok = false; break
                    }
                    L[i, i] = sqrt(s)
                } else {
                    L[i, j] = s / L[j, j]
                }
            }
            if (!ok) break
        }
        if (ok) return L
        jitter *= 100.0
    }
    error("cholesky failed even with jitter")
}

fun solveSPD(A: Mat, b: DoubleArray): DoubleArray {
    val L = cholesky(A)
    val n = A.r
    val y = DoubleArray(n)
    for (i in 0 until n) {
        var s = b[i]
        for (k in 0 until i) s -= L[i, k] * y[k]
        y[i] = s / L[i, i]
    }
    val x = DoubleArray(n)
    for (i in n - 1 downTo 0) {
        var s = y[i]
        for (k in i + 1 until n) s -= L[k, i] * x[k]
        x[i] = s / L[i, i]
    }
    return x
}

fun invSPD(A: Mat): Mat {
    val n = A.r
    val out = Mat(n, n)
    val L = cholesky(A)
    val y = DoubleArray(n)
    for (col in 0 until n) {
        for (i in 0 until n) y[i] = if (i == col) 1.0 else 0.0
        for (i in 0 until n) {
            var s = y[i]
            for (k in 0 until i) s -= L[i, k] * y[k]
            y[i] = s / L[i, i]
        }
        for (i in n - 1 downTo 0) {
            var s = y[i]
            for (k in i + 1 until n) s -= L[k, i] * y[k]
            y[i] = s / L[i, i]
        }
        for (i in 0 until n) out[i, col] = y[i]
    }
    return out.symmetrize()
}

/** 数值稳定的 2x2 马氏距离; 协方差退化时返回一个很大的数而不是 NaN */
fun mahalanobis2(z: DoubleArray, S: Mat2xx, floorVar: Double = 1e-6): Double {
    val a = max(S.a, floorVar)
    val d = max(S.d, floorVar)
    val m = Mat2xx(a, S.b, d)
    val inv = m.inv() ?: return Double.MAX_VALUE / 4
    return z[0] * (inv.a * z[0] + inv.b * z[1]) + z[1] * (inv.b * z[0] + inv.d * z[1])
}

/** 角度归一化到 (-pi, pi] */
fun wrapPi(a: Double): Double {
    var x = a
    while (x > Math.PI) x -= 2 * Math.PI
    while (x <= -Math.PI) x += 2 * Math.PI
    return x
}

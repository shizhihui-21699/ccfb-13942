package io.gnsstrust.core

import kotlin.math.*

/**
 * 局部切平面 (ENU, 东-北-天), 米为单位。
 *
 * 全模块内部只用局部 ENU —— 卫星定位的纬度/经度只在接口边界出现。
 * 这样做既避免了大数相减的精度损失, 也让协方差矩阵有物理量纲。
 */
class LocalFrame(val lat0Deg: Double, val lon0Deg: Double) {

    private val a = 6378137.0
    private val f = 1.0 / 298.257223563
    private val e2 = f * (2 - f)
    private val lat0 = Math.toRadians(lat0Deg)
    private val lon0 = Math.toRadians(lon0Deg)

    /** 子午圈/卯酉圈曲率半径 */
    private val w = sqrt(1 - e2 * sin(lat0).pow(2))
    private val rM = a * (1 - e2) / (w * w * w)
    private val rN = a / w

    fun toEnu(latDeg: Double, lonDeg: Double): DoubleArray {
        val dLat = Math.toRadians(latDeg) - lat0
        val dLon = Math.toRadians(lonDeg) - lon0
        return doubleArrayOf(dLon * rN * cos(lat0), dLat * rM)
    }

    fun toLatLon(e: Double, n: Double): DoubleArray =
        doubleArrayOf(
            lat0Deg + Math.toDegrees(n / rM),
            lon0Deg + Math.toDegrees(e / (rN * cos(lat0)))
        )
}

/**
 * 罗盘方位 -> 数学角。
 * 航向角 psi 按数学惯例: 0 = 东, 逆时针为正, 单位弧度。
 * 罗盘 heading 0 = 北, 顺时针为正。
 */
fun compassToPsi(headingDeg: Double): Double = wrapPi(Math.toRadians(90.0 - headingDeg))

fun psiToCompass(psi: Double): Double {
    var h = Math.toDegrees(PI / 2 - psi)
    h %= 360.0
    if (h < 0) h += 360.0
    return h
}

/** 折线路径上的最近点投影, 用于把轨迹吸附到路网上 (地图约束检测要用) */
object Polyline {
    data class Projection(val dist: Double, val sAlong: Double, val segIdx: Int, val t: Double)

    fun project(pts: List<DoubleArray>, p: DoubleArray): Projection {
        var best = Projection(Double.MAX_VALUE, 0.0, -1, 0.0)
        var acc = 0.0
        for (i in 0 until pts.size - 1) {
            val a = pts[i]; val b = pts[i + 1]
            val dx = b[0] - a[0]; val dy = b[1] - a[1]
            val len2 = dx * dx + dy * dy
            val len = sqrt(len2)
            if (len < 1e-9) continue
            var t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2
            t = t.coerceIn(0.0, 1.0)
            val qx = a[0] + t * dx; val qy = a[1] + t * dy
            val d = hypot(p[0] - qx, p[1] - qy)
            if (d < best.dist) best = Projection(d, acc + t * len, i, t)
            acc += len
        }
        return best
    }

    /** 沿路径前进 s 米处的点与切向 */
    fun at(pts: List<DoubleArray>, s: Double): DoubleArray {
        var acc = 0.0
        for (i in 0 until pts.size - 1) {
            val a = pts[i]; val b = pts[i + 1]
            val dx = b[0] - a[0]; val dy = b[1] - a[1]
            val len = hypot(dx, dy)
            if (len < 1e-9) continue
            if (acc + len >= s || i == pts.size - 2) {
                val t = ((s - acc) / len).coerceIn(0.0, 1.0)
                return doubleArrayOf(
                    a[0] + t * dx, a[1] + t * dy,
                    dx / len, dy / len
                )
            }
            acc += len
        }
        return doubleArrayOf(0.0, 0.0, 1.0, 0.0)
    }

    fun length(pts: List<DoubleArray>): Double {
        var acc = 0.0
        for (i in 0 until pts.size - 1)
            acc += hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        return acc
    }
}

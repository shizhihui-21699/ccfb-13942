package io.gnsstrust.cli

import io.gnsstrust.detect.DetectorConfig
import io.gnsstrust.detect.Thresholds
import io.gnsstrust.eval.*
import io.gnsstrust.model.EnvClass
import io.gnsstrust.model.Platform
import io.gnsstrust.sim.*
import java.io.File
import kotlin.math.*

/**
 * 命令行入口。
 *
 *   ./gradlew run --args="demo"        单场景带解说的演示
 *   ./gradlew evalReport               跑完整评估, 写 docs/RESULTS.md
 *   ./gradlew run --args="quick"       快速自检
 */
fun main(args: Array<String>) {
    when (args.getOrNull(0) ?: "demo") {
        "demo" -> demo()
        "quick" -> quick()
        "eval" -> {
            val out = args.getOrNull(1) ?: "docs/RESULTS.md"
            ReportWriter.writeFull(File(out))
            println("written: $out")
        }
        "sweep" -> sweepOnly()
        else -> {
            println("usage: demo | quick | eval [out.md] | sweep")
        }
    }
}

// ---------------------------------------------------------------- 门槛自证

private fun gateSelfTest() {
    println("=".repeat(86))
    println("A. 门槛是怎么算出来的")
    println("=".repeat(86))
    val g = Thresholds.SINGLE_POINT_CHI2
    println("单点卡方门槛 (2 自由度, α=1e-4): χ² = %.3f".format(g))
    println("  => 检验统计量 (马氏距离平方) 超过 %.2f 判为异常。".format(g))
    println("  => 对应单次虚警概率 1.0e-4 (每 10000 帧一次)。".format())
    println()
    println("CUSUM 门槛 (按环境反解, 每 τ 秒推进一次):")
    println("%-18s %8s %12s %8s %8s %14s %16s".format("环境", "相关τ(s)", "ARL0(有效样本)", "k", "h", "可检出δ(σ)", "≈误报间隔(h)"))
    for (env in EnvClass.entries) {
        val p = Thresholds.cusumFor(env, 0.1, 1.0, 0.35, when (env) {
            EnvClass.OPEN_SKY -> 2.0; EnvClass.SUBURBAN -> 5.0
            EnvClass.URBAN_CANYON -> 45.0; else -> 60.0
        })
        println("%-18s %8.1f %12.0f %8.3f %8.3f %14.3f %16.2f".format(
            env.name, p.effTau, p.arl0, p.kRef, p.h, p.minDetectableDelta(),
            p.arl0 * p.effTau / 3600.0
        ))
    }
    println()
    println("含义: ARL0 是「平均多少个**独立样本**才误报一次」。城市峡谷的误差")
    println("      相关时间 45 s, 因此 45 s 才有一个独立样本 —— 折算成误报间隔仍是 1 h。")
    println("      若错误地每帧 (0.1 s) 都推进, 误报间隔会缩短 450 倍。这正是要防的坑。")
    println()
}

// ---------------------------------------------------------------- 演示

private fun demo() {
    gateSelfTest()

    println("=".repeat(86))
    println("B. 场景演示: 卡车进城, 攻击者在出城后开始 1.5 m/s 缓慢拉偏 (一致欺骗)")
    println("=".repeat(86))
    val atk = GradualDrift(onset = 40.0, rateMps = 1.5, maxOffsetM = 900.0, consistent = true, dirDeg = 15.0)
    val meta = World.build(ScenarioSpec("demo_drift", Platform.TRUCK, atk, 42L, 700.0))
    val det = io.gnsstrust.detect.SpoofDetector()
    det.attachMap(meta.roads)
    det.anchorPositions = AnchorNetwork.forTruck(42L).associate { it.id to doubleArrayOf(it.e, it.n) }

    println("%6s %8s %10s %10s %9s %16s %s".format(
        "t(s)", "环境", "真实(偏)", "评分", "判决", "主证据", "说明"))
    var lastPrint = -10.0
    for (i in meta.frames.indices) {
        val f = meta.frames[i]
        val tr = meta.truth[i]
        val a = meta.attack[i]
        val rep = det.update(f)
        if (f.t - lastPrint >= 10.0) {
            lastPrint = f.t
            println("%6.0f %8s %10.1f %10.2f %9s %16s %s".format(
                f.t, rep.env.name.take(8), a.horizontal, rep.score,
                rep.verdict.name.take(9),
                rep.dominant?.name?.take(16) ?: "-",
                rep.reasons.firstOrNull()?.take(40) ?: ""
            ))
        }
    }
    println()
    println("=".repeat(86))
    println("C. 隧道场景: 同样的攻击在隧道区间内")
    println("=".repeat(86))
    demoOutage()
}

private fun demoOutage() {
    // 让攻击恰好在隧道里开始
    val sc = TruckScene(7L)
    // 攻击在【隧道里】开始: 此时 GNSS 已失锁, 攻击者以为"没人看得见"
    val atk = GradualDrift(onset = 505.0, rateMps = 1.0, maxOffsetM = 600.0, consistent = true)
    val meta = World.build(ScenarioSpec("demo_outage", Platform.TRUCK, atk, 7L, 700.0))
    println("失锁区间: " + meta.outageWindows.joinToString { "%.0f-%.0f s".format(it.start, it.endInclusive) })
    val det = io.gnsstrust.detect.SpoofDetector()
    det.attachMap(meta.roads)
    det.anchorPositions = AnchorNetwork.forTruck(7L).associate { it.id to doubleArrayOf(it.e, it.n) }
    println("%6s %8s %10s %10s %9s".format("t(s)", "环境", "真实(偏)", "评分", "判决"))
    var last = -10.0
    for (i in meta.frames.indices) {
        val f = meta.frames[i]
        val rep = det.update(f)
        if (f.t - last >= 10.0) {
            last = f.t
            println("%6.0f %8s %10.1f %10.2f %9s".format(
                f.t, rep.env.name.take(8), meta.attack[i].horizontal, rep.score, rep.verdict.name.take(9)))
        }
    }
    println()
}

// ---------------------------------------------------------------- 快速自检

private fun quick() {
    println("quick check: 3 个场景各跑一次")
    val cases = listOf(
        Triple("clean", NoAttack, Platform.TRUCK),
        Triple("drift1.5", GradualDrift(20.0, 1.5, 2000.0, true), Platform.TRUCK),
        Triple("jump", JumpTakeover(20.0, 300.0, 120.0), Platform.TRUCK),
    )
    for ((n, a, p) in cases) {
        val meta = World.build(ScenarioSpec(n, p, a, 11L, 180.0))
        val r = Evaluator.run(meta)
        println("%-12s frames=%d confirmed=%s delay=%.1f radius=%.1f trusted=%.2f".format(
            n, r.frames.size, r.firstConfirmed, r.detectDelayS, r.detectionRadiusM, r.trustedFraction()))
    }
}

private fun sweepOnly() {
    println("sweep: 卡车, 一致欺骗, 速率扫描")
    val rows = Experiments.driftSweep(
        Platform.TRUCK, listOf(0.2, 0.5, 1.0, 2.0, 5.0), trials = 3, durationS = 300.0, consistent = true
    )
    for (r in rows) {
        println("%.1f m/s (%5.0f m/min): detect %d/%d, delay %.0fs, radius %.0f m (p90 %.0f)".format(
            r.rateMps, r.rateMperMin, r.detected, r.trials, r.medianDelayS, r.medianRadiusM, r.p90RadiusM))
    }
}

package io.gnsstrust.cli

import io.gnsstrust.detect.DetectorId
import io.gnsstrust.eval.*
import io.gnsstrust.model.EnvClass
import io.gnsstrust.model.Platform
import io.gnsstrust.detect.Thresholds
import io.gnsstrust.sim.*
import java.io.File
import kotlin.math.*

/**
 * 生成 docs/RESULTS.md —— 里面所有数字都是本次运行实际跑出来的,
 * 不是手抄的。这样重跑就能验证结论是否还成立。
 */
object ReportWriter {

    fun writeFull(out: File) {
        val sb = StringBuilder()
        val t0 = System.currentTimeMillis()

        sb.appendLine("# 评估结果")
        sb.appendLine()
        sb.appendLine("> 本文件由 `./gradlew evalReport` 生成, 所有数字均为当次实测。")
        sb.appendLine("> 仿真种子固定, 可复现。")
        sb.appendLine()

        // ---------------- 门槛
        sb.appendLine("## 1. 门槛的推导")
        sb.appendLine()
        sb.appendLine("单点一致性检验使用马氏距离平方, 在无攻击时服从 χ²(2)。")
        sb.appendLine("取单次虚警率 α = 1e-4, 门槛 = **%.3f**。".format(Thresholds.SINGLE_POINT_CHI2))
        sb.appendLine()
        sb.appendLine("序列检验用 CUSUM, 参考值 k = delta_star/2, 门槛 h 由目标 ARL0 反解:")
        sb.appendLine()
        sb.appendLine("| 环境 | 相关时间 τ (s) | ARL0 (帧) | k | h | 最小可检出 δ (σ/帧) |")
        sb.appendLine("|---|---|---|---|---|---|")
        for (env in EnvClass.entries) {
            val tau = when (env) {
                EnvClass.OPEN_SKY -> 2.0; EnvClass.SUBURBAN -> 5.0
                EnvClass.URBAN_CANYON -> 45.0; else -> 60.0
            }
            val p = Thresholds.cusumFor(env, 0.1, 1.0, 0.35, tau)
            sb.appendLine("| ${env.name} | %.1f | %.0f | %.3f | %.3f | %.3f |".format(
                tau, p.arl0, p.kRef, p.h, p.minDetectableDelta()))
        }
        sb.appendLine()

        // ---------------- 虚警
        sb.appendLine("## 2. 干净数据虚警率 (无攻击)")
        sb.appendLine()
        sb.appendLine("| 平台 | 时长 | 试验数 | CONFIRMED 次数 | 折算/24h | SUSPECT 次数 | 折算/24h | 判 TRUSTED 比例 | 位置不确定度中位数 |")
        sb.appendLine("|---|---|---|---|---|---|---|---|---|")
        val fa = ArrayList<FalseAlarmResult>()
        for (p in listOf(Platform.TRUCK, Platform.DRONE)) {
            val r = Experiments.falseAlarmStudy(p, trials = 12, durationS = 600.0)
            fa.add(r)
            sb.appendLine("| ${p.name} | %.0f min | ${r.trials} | ${r.confirmedRuns} | %.2f | ${r.suspectRuns} | %.2f | %.3f | %.1f m |".format(
                r.durationS / 60, r.confirmedPer24h, r.suspectPer24h, r.medianTrustedFraction, r.medianUncertaintyM))
        }
        sb.appendLine()

        // ---------------- 速率扫描
        sb.appendLine("## 3. 检出半径 vs 拉偏速率 (核心结果)")
        sb.appendLine()
        sb.appendLine("**问题: 被拉偏多远时能发现?**")
        sb.appendLine()
        sb.appendLine("每一行: 给定匀速拉偏速率, 蒙特卡洛跑 N 次, 统计第一次确认被欺骗时")
        sb.appendLine("GNSS 报告位置与真实位置的距离 (\"检出半径\")。")
        sb.appendLine()
        val rates = listOf(0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0)
        for (p in listOf(Platform.TRUCK, Platform.DRONE)) {
            sb.appendLine("### ${if (p == Platform.TRUCK) "冷链卡车" else "巡检无人机"} — 一致欺骗 (伪造 Doppler)")
            sb.appendLine()
            sb.appendLine("| 拉偏速率 | m/min | 检出率 | 平均延迟 | 检出半径(中位) | 检出半径(90分位) |")
            sb.appendLine("|---|---|---|---|---|---|")
            val rows = Experiments.driftSweep(p, rates, trials = 6, durationS = 420.0, consistent = true)
            for (r in rows) {
                sb.appendLine("| %.2f m/s | %.0f | %d/%d | %s | %s | %s |".format(
                    r.rateMps, r.rateMperMin, r.detected, r.trials,
                    fmtS(r.medianDelayS), fmtM(r.medianRadiusM), fmtM(r.p90RadiusM)))
            }
            sb.appendLine()
            sb.appendLine("### ${p.name} — 只偏位置 (不伪造 Doppler)")
            sb.appendLine()
            sb.appendLine("| 拉偏速率 | m/min | 检出率 | 平均延迟 | 检出半径(中位) | 检出半径(90分位) |")
            sb.appendLine("|---|---|---|---|---|---|")
            val rows2 = Experiments.driftSweep(p, rates, trials = 6, durationS = 420.0, consistent = false)
            for (r in rows2) {
                sb.appendLine("| %.2f m/s | %.0f | %d/%d | %s | %s | %s |".format(
                    r.rateMps, r.rateMperMin, r.detected, r.trials,
                    fmtS(r.medianDelayS), fmtM(r.medianRadiusM), fmtM(r.p90RadiusM)))
            }
            sb.appendLine()
        }

        // ---------------- 攻击矩阵
        sb.appendLine("## 4. 各攻击型态的检出能力")
        sb.appendLine()
        sb.appendLine("| 平台 | 攻击 | 检出率 | 延迟 | 检出半径 |")
        sb.appendLine("|---|---|---|---|---|")
        for (p in listOf(Platform.TRUCK, Platform.DRONE)) {
            val rows = Experiments.attackMatrix(p, trials = 6, durationS = 400.0)
            for (r in rows) {
                sb.appendLine("| ${p.name} | ${r.attack} | %d/%d | %s | %s |".format(
                    r.detected, r.trials, fmtS(r.medianDelayS), fmtM(r.medianRadiusM)))
            }
        }
        sb.appendLine()

        // ---------------- 基线对比
        sb.appendLine("## 5. 与朴素基线的对比")
        sb.appendLine()
        sb.appendLine("慢速一致欺骗 (1.5 m/s), 这是接收机自检完全看不出来的情形。")
        sb.appendLine()
        sb.appendLine("| 方法 | 检出率 | 说明 |")
        sb.appendLine("|---|---|---|")
        for (p in listOf(Platform.TRUCK, Platform.DRONE)) {
            for (r in Experiments.baselineComparison(p, trials = 6, durationS = 400.0)) {
                sb.appendLine("| ${p.name} / ${r.method} | %d/%d | ${r.note} |".format(r.detected, r.trials))
            }
        }
        sb.appendLine()

        // ---------------- 隧道
        sb.appendLine("## 6. 隧道内的行为")
        sb.appendLine()
        sb.appendLine("隧道里 GNSS 不可用。检测器必须:")
        sb.appendLine("1. 不产生任何位置类证据 (否则每次过隧道都误报)")
        sb.appendLine("2. 输出 DEGRADED, 而不是 TRUSTED")
        sb.appendLine("3. 出隧道后按失锁时长放大门槛再恢复检验")
        sb.appendLine()
        sb.appendLine(tunnelSummary())

        sb.appendLine()
        sb.appendLine("---")
        sb.appendLine()
        sb.appendLine("生成耗时 %.1f s".format((System.currentTimeMillis() - t0) / 1000.0))

        out.parentFile?.mkdirs()
        out.writeText(sb.toString())
    }

    private fun tunnelSummary(): String {
        val sb = StringBuilder()
        // 找一段含隧道的真值轨迹, 统计检测器在隧道中的判决分布
        val meta = World.build(ScenarioSpec(
            "tunnel_check", Platform.TRUCK, NoAttack, 21L, 420.0,
            forceOutage = listOf(120.0..210.0),
        ))
        val det = io.gnsstrust.detect.SpoofDetector()
        det.anchorPositions = AnchorNetwork.forTruck(21L).associate { it.id to doubleArrayOf(it.e, it.n) }
        val counts = HashMap<String, Int>()
        for (i in meta.frames.indices) {
            val rep = det.update(meta.frames[i])
            if (meta.frames[i].t in 120.0..210.0)
                counts.merge(rep.verdict.name, 1, Int::plus)
        }
        val degradedFrac = (counts["DEGRADED"] ?: 0).toDouble() /
            max(1, counts.values.sum())
        sb.appendLine("强制失锁 120-210 s 期间的判决分布: " +
            counts.entries.joinToString { "${it.key}=${it.value}" })
        sb.appendLine()
        sb.appendLine("其中 DEGRADED 占比 %.1f%% (门限 ≥ 90%%: %s)".format(
            degradedFrac * 100, if (degradedFrac >= 0.9) "通过" else "**未通过**"))
        sb.appendLine()
        sb.appendLine()
        sb.appendLine("隧道期间的判决应为 DEGRADED (不知道), 而不是 TRUSTED (没问题)。")
        sb.appendLine("位置不确定度会随失锁时长增长, 这是设计意图 —— 调度系统据此可以知道")
        sb.appendLine("这段时间的轨迹不能用来结算工时。")
        return sb.toString()
    }

    private fun fmtS(x: Double) = if (x.isNaN()) "—" else "%.0f s".format(x)
    private fun fmtM(x: Double) = if (x.isNaN()) "—" else "%.0f m".format(x)
}

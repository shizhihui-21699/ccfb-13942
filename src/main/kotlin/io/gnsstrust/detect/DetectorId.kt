package io.gnsstrust.detect

/**
 * 每一路检验都有独立编号 —— 报告里要能说清"是哪个证据抓到的",
 * 以及"哪一路在干净数据上误报了", 否则无法调参。
 */
enum class DetectorId(val title: String) {
    /* ---- 与 GNSS 位置/速度的一致性 ---- */
    POS_VS_INS("GNSS 位置 vs 惯性航位推算"),
    VEL_VS_INS("GNSS 多普勒速度 vs 轮速/惯导"),
    COURSE_VS_HEADING("GNSS 航向 vs 罗盘/陀螺航向"),
    SPEED_VS_WHEEL("GNSS 速度量级 vs 轮速"),

    /* ---- 绝对参照 (与 GNSS 物理独立) ---- */
    RANGE_VS_INS("独立测距 (蜂窝 TA / C2 链路) vs 惯导"),
    MAP_VS_INS("地图约束 (路网/禁飞区)"),
    ALT_VS_BARO("GNSS 高度 vs 气压/测距高度"),

    /* ---- 射频层信号取证 ---- */
    RF_ANOMALY("射频指纹异常 (AGC/C-N0/星座)"),
    RX_SELFCHECK("接收机自检 (残差/钟差/RAIM)"),

    /* ---- 行为层 ---- */
    MOTION_PLAUSIBILITY("运动学不可能 (加速度/超速/静止)"),
    TIME_SYNC("时间一致性 (伪距率 vs 位置率)"),
}

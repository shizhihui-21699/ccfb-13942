plugins {
    kotlin("jvm") version "2.1.0"
    application
}

group = "io.gnsstrust"
version = "0.1.0"

repositories { mavenCentral() }

kotlin { jvmToolchain(21) }

dependencies {
    testImplementation("org.junit.jupiter:junit-jupiter:5.11.4")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.test {
    useJUnitPlatform()
    maxHeapSize = "2g"
    testLogging {
        events("passed", "skipped", "failed")
        showStandardStreams = false
    }
}

application { mainClass.set("io.gnsstrust.cli.MainKt") }

// 生成评估报告: ./gradlew evalReport
tasks.register<JavaExec>("evalReport") {
    group = "verification"
    description = "跑蒙特卡洛评估, 生成 docs/RESULTS.md"
    classpath = sourceSets["main"].runtimeClasspath
    mainClass.set("io.gnsstrust.cli.MainKt")
    args("eval", "docs/RESULTS.md")
    maxHeapSize = "3g"
}

tasks.register<JavaExec>("demo") {
    group = "application"
    description = "跑一个带解说的单场景演示"
    classpath = sourceSets["main"].runtimeClasspath
    mainClass.set("io.gnsstrust.cli.MainKt")
    args("demo")
}

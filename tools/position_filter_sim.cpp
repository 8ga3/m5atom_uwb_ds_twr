// フィルタ (src/common/position_filter.h) と最小二乗 (src/common/trilateration.h) を、
// 同じ模擬の測距に掛けて比べるホスト用のシミュレーション
// (doc/multi-anchor-positioning-design.md 3.7)。
//
// ビルドと実行 (リポジトリのルートで):
//   c++ -std=c++17 -O2 -I src/common tools/position_filter_sim.cpp -o /tmp/position_filter_sim
//   /tmp/position_filter_sim [速度 m/s] [外れ値の割合] [乱数の種]
// プロセスノイズを変えるときは、ファームと同じく -D UWB_KF_ACCEL_PSD=<値> を付けてビルドする。
//
// 条件:
// - アンカー 4 台を 5 m x 4 m の四隅、高さ 1.8 m に置き、タグの高さは 0.2 m
// - 10Hz、1 スロット 25 ms。測距には標準偏差 20 mm の雑音と、アンカーごとの一定のずれを足す
// - 指定した割合の測距に +0.6 m の外れ値 (反射波を拾った場合を想定) を足す
// - タグは中心 (2.5, 2.0)、半径 1.5 m の円を指定した速度で回る (0 なら円周上の 1 点に止まる)
// - 60 秒のうち、周期 200〜219 (2 秒) は全アンカーを遮蔽し、周期 300〜339 (4 秒) は 2 台を遮蔽する
// - 誤差は、どちらも周期の最後のスロットの時刻の真値に対して測る
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

#include "position_filter.h"
#include "trilateration.h"

namespace {

constexpr int CYCLES             = 600;
constexpr uint32_t CYCLE_MS      = 100;
constexpr uint32_t SLOT_MS       = 25;
constexpr size_t ANCHORS         = 4;
constexpr float TAG_Z            = 0.2f;
constexpr float CIRCLE_X         = 2.5f;
constexpr float CIRCLE_Y         = 2.0f;
constexpr float CIRCLE_R         = 1.5f;
constexpr float NOISE_SIGMA_M    = 0.02f;
constexpr float OUTLIER_M        = 0.6f;
constexpr int BLACKOUT_FIRST     = 200;  // 全アンカーを遮蔽する周期
constexpr int BLACKOUT_LAST      = 219;
constexpr int PARTIAL_FIRST      = 300;  // 0x0101 と 0x0102 にあたる 2 台を遮蔽する周期
constexpr int PARTIAL_LAST       = 339;

constexpr float ANCHOR_X[ANCHORS] = {0.0f, 5.0f, 5.0f, 0.0f};
constexpr float ANCHOR_Y[ANCHORS] = {0.0f, 0.0f, 4.0f, 4.0f};
constexpr float ANCHOR_Z[ANCHORS] = {1.8f, 1.8f, 1.8f, 1.8f};
// 実機のテーブルの上の試験で、4 台とも巻尺より 18〜70 mm 短かった (3.6) のに合わせる。
constexpr float ANCHOR_BIAS[ANCHORS] = {-0.020f, -0.070f, -0.040f, -0.018f};

struct Truth {
    float x;
    float y;
};

Truth truthAt(uint32_t tMs, float speed)
{
    const float angle = (speed / CIRCLE_R) * (static_cast<float>(tMs) / 1000.0f);
    return {CIRCLE_X + (CIRCLE_R * cosf(angle)), CIRCLE_Y + (CIRCLE_R * sinf(angle))};
}

bool blocked(int cycle, size_t anchor)
{
    if ((cycle >= BLACKOUT_FIRST) && (cycle <= BLACKOUT_LAST)) return true;
    return (cycle >= PARTIAL_FIRST) && (cycle <= PARTIAL_LAST) && ((anchor == 1) || (anchor == 2));
}

// 真値との差 (推定 - 真値) を数える。静止時は、平均の差 (偏り) と、自分の平均位置からの
// 散らばりを分けて見られるようにする。
struct ErrorStat {
    double sumSq = 0.0;
    double sumDx = 0.0;
    double sumDy = 0.0;
    double max   = 0.0;
    int count    = 0;

    void add(double dx, double dy)
    {
        const double error = std::hypot(dx, dy);
        sumSq += error * error;
        sumDx += dx;
        sumDy += dy;
        if (error > max) max = error;
        ++count;
    }

    double rms() const
    {
        return (count > 0) ? std::sqrt(sumSq / count) : 0.0;
    }

    double meanDx() const
    {
        return (count > 0) ? sumDx / count : 0.0;
    }

    double meanDy() const
    {
        return (count > 0) ? sumDy / count : 0.0;
    }

    // 自分の平均位置からの散らばりの RMS
    double scatter() const
    {
        if (count == 0) return 0.0;
        const double v = (sumSq / count) - (meanDx() * meanDx()) - (meanDy() * meanDy());
        return (v > 0.0) ? std::sqrt(v) : 0.0;
    }
};

}  // namespace

int main(int argc, char** argv)
{
    const float speed       = (argc > 1) ? std::strtof(argv[1], nullptr) : 1.0f;
    const float outlierRate = (argc > 2) ? std::strtof(argv[2], nullptr) : 0.01f;
    const unsigned seed     = (argc > 3) ? static_cast<unsigned>(std::strtoul(argv[3], nullptr, 10)) : 1;

    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0f, NOISE_SIGMA_M);
    std::uniform_real_distribution<float> uniform(0.0f, 1.0f);

    PositionFilter filter;
    ErrorStat lsAll, kfAll, kfPartial;
    int outliers = 0, rejected = 0, inits = 0, predicted = 0, invalid = 0;
    int lsPartial = 0;
    int blackoutValidCycles = 0, resetCycle = -1, reinitCycle = -1;

    for (int c = 0; c < CYCLES; ++c) {
        const uint32_t startMs = 1000 + (static_cast<uint32_t>(c) * CYCLE_MS);
        TrilatInput lsInputs[ANCHORS];
        PositionFilterRange kfInputs[ANCHORS];
        size_t n = 0;
        for (size_t a = 0; a < ANCHORS; ++a) {
            if (blocked(c, a)) continue;
            const uint32_t tMs = startMs + (static_cast<uint32_t>(a) * SLOT_MS);
            const Truth truth  = truthAt(tMs, speed);
            const float dx     = truth.x - ANCHOR_X[a];
            const float dy     = truth.y - ANCHOR_Y[a];
            const float dz     = TAG_Z - ANCHOR_Z[a];
            float range        = sqrtf((dx * dx) + (dy * dy) + (dz * dz)) + ANCHOR_BIAS[a] + noise(rng);
            if (uniform(rng) < outlierRate) {
                range += OUTLIER_M;
                ++outliers;
            }
            lsInputs[n] = {ANCHOR_X[a], ANCHOR_Y[a], ANCHOR_Z[a], range};
            kfInputs[n] = {ANCHOR_X[a], ANCHOR_Y[a], ANCHOR_Z[a], range, tMs};
            ++n;
        }

        const uint32_t endMs   = startMs + (static_cast<uint32_t>(ANCHORS - 1) * SLOT_MS);
        const TrilatResult ls  = trilaterate2d(lsInputs, n, TAG_Z);
        const bool lsOk        = (ls.status == TrilatStatus::Ok);
        const PositionFilterCycle kf =
            filter.step(kfInputs, n, TAG_Z, endMs, lsOk, ls.x, ls.y, ls.residualRms, ls.used);
        const Truth truth = truthAt(endMs, speed);

        const bool partial = (c >= PARTIAL_FIRST) && (c <= PARTIAL_LAST);
        if (lsOk) {
            lsAll.add(ls.x - truth.x, ls.y - truth.y);
            if (partial) ++lsPartial;
        }
        if (kf.valid) {
            const double dx = kf.x - truth.x;
            const double dy = kf.y - truth.y;
            kfAll.add(dx, dy);
            if (partial) kfPartial.add(dx, dy);
            if (!kf.updated) ++predicted;
        } else {
            ++invalid;
        }
        if ((c >= BLACKOUT_FIRST) && (c <= BLACKOUT_LAST) && kf.valid) ++blackoutValidCycles;
        if ((resetCycle < 0) && (c >= BLACKOUT_FIRST) && (kf.reset != PositionFilterReset::None)) {
            resetCycle = c;
            std::printf("blackout: reset at cycle %d (%s)\n", c, positionFilterResetName(kf.reset));
        }
        if ((reinitCycle < 0) && (c > BLACKOUT_LAST) && kf.initialized) {
            reinitCycle = c;
            std::printf("blackout: reinitialized at cycle %d\n", c);
        }
        rejected += kf.rejected;
        inits += kf.initialized ? 1 : 0;
    }

    std::printf("speed=%.1f m/s outlier_rate=%.3f seed=%u accel_psd=%.2f\n", static_cast<double>(speed),
                static_cast<double>(outlierRate), seed, static_cast<double>(PFILTER_ACCEL_PSD));
    std::printf("least squares: rms=%.0f mm max=%.0f mm fixes=%d\n", lsAll.rms() * 1000.0, lsAll.max * 1000.0,
                lsAll.count);
    std::printf("filter: rms=%.0f mm max=%.0f mm valid=%d predicted=%d invalid=%d inits=%d\n",
                kfAll.rms() * 1000.0, kfAll.max * 1000.0, kfAll.count, predicted, invalid, inits);
    std::printf("outliers=%d rejected=%d\n", outliers, rejected);
    // 静止時に意味を持つ。走行時は真値の移動に対する遅れが偏りと散らばりに入る。
    std::printf("mean error: least squares=(%.1f, %.1f) mm scatter=%.1f mm, filter=(%.1f, %.1f) mm scatter=%.1f mm\n",
                lsAll.meanDx() * 1000.0, lsAll.meanDy() * 1000.0, lsAll.scatter() * 1000.0, kfAll.meanDx() * 1000.0,
                kfAll.meanDy() * 1000.0, kfAll.scatter() * 1000.0);
    std::printf("two anchors blocked: least squares fixes=%d, filter valid=%d rms=%.0f mm max=%.0f mm\n", lsPartial,
                kfPartial.count, kfPartial.rms() * 1000.0, kfPartial.max * 1000.0);
    std::printf("all anchors blocked: filter valid for %d of %d cycles\n", blackoutValidCycles,
                BLACKOUT_LAST - BLACKOUT_FIRST + 1);
    return 0;
}

// 2D 三辺測量 (高さ既知の最小二乗)。
//
// タグの高さを固定値として与え、各アンカーまでの測距値 (斜距離) から水平面内の
// 位置 (x, y) を求める (doc/multi-anchor-positioning-design.md 3.3 / 3.5)。
// アンカーは天井付近、タグはローバーの上と高さが大きく違うので、測距値をそのまま
// 水平距離として扱うと位置が外側へずれる。ここでは高さの差を含めた 3 次元の距離の
// 式に対して、x と y だけを未知数にした最小二乗を解く。
//
// 解き方は 2 段階にしている。
// 1. 線形化した最小二乗で初期値を作る。各アンカーの式から先頭のアンカーの式を
//    引くと 2 乗の項が消え、x と y の 1 次式になる。雑音が無ければこれだけで解けるが、
//    雑音があると先頭のアンカーの誤差がすべての式に乗り、最小二乗としては偏る
// 2. その初期値から Gauss-Newton 法で、測距値との差 (残差) の 2 乗和を最小にする
//
// Arduino にも M5 のライブラリにも依存させない。ホストの C++ コンパイラで
// そのまま試験できるようにするためである。使うのは TAG だけなので、他の
// common/ のヘッダと同じくヘッダのみで実装する (platformio.ini 参照)。
//
// 計算は float で行う。ESP32 / ESP32-S3 の FPU は単精度だけで、double はソフトウェアの
// 演算になる。座標はアンカーの重心を原点にずらしてから解くので、実験場の広さ
// (数十 m) なら float の精度 (1000 m でも 0.1 mm 以下) で足りる。
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

// 解に使えるアンカーの上限。巡回の上限 (server_config.h の UWB_ANCHOR_MAX) より
// 大きければよい。ここで server_config.h を include すると Arduino に依存するので、
// 呼び出し側で static_assert して揃える。
static constexpr size_t TRILAT_ANCHOR_MAX = 16;

// 2D で解くのに必要な測距の本数。2 本でも円の交点として解けるが、交点が 2 つあって
// どちらか決まらないので 3 本を下限にする。
static constexpr size_t TRILAT_MIN_RANGES = 3;

// Gauss-Newton の反復の上限と、収束とみなす 1 回の更新量 (メートル)。
// 線形化した解から始めると、通常は 2〜3 回で更新量が 0.1 mm を下回る。
static constexpr int TRILAT_MAX_ITERATIONS = 10;
static constexpr float TRILAT_CONVERGED_M  = 0.0001f;

// 配置の悪さ (ほぼ一直線に並んだアンカー、真上に近いタグなど) を判定するしきい値。
// 正規方程式の 2x2 行列 A について det(A) / (trace(A) / 2)^2 を見る。固有値を
// λ1 >= λ2 とすると、この値は 4 λ1 λ2 / (λ1 + λ2)^2 で、λ2 が λ1 より十分小さいときは
// 4 λ2 / λ1 に近い。0.001 は条件数でおよそ 4000 にあたり、弱い方向の誤差が強い方向の
// 約 60 倍に増幅される配置を解けないものとして扱う。
static constexpr float TRILAT_MIN_GEOMETRY = 0.001f;

struct TrilatInput {
    float x;      // アンカーの設置座標 (メートル)
    float y;
    float z;
    float range;  // そのアンカーまでの測距値 (メートル。バイアス補正済み)
};

enum class TrilatStatus : uint8_t {
    Ok,
    TooFewRanges,  // 使える測距が TRILAT_MIN_RANGES 本に満たない
    BadGeometry,   // アンカーの配置が悪く、2 方向を区別できない
    NonFinite,     // 計算の途中で NaN / 無限大が出た
};

struct TrilatResult {
    TrilatStatus status;
    float x;            // 推定した位置 (メートル)。status が Ok のときだけ意味を持つ
    float y;
    float residualRms;  // 最後の位置での残差の RMS (メートル)
    uint8_t used;       // 解に使った測距の本数
    uint8_t iterations; // Gauss-Newton の反復回数
    bool converged;     // 反復の上限までに更新量が TRILAT_CONVERGED_M を下回ったか
};

static const char* trilatStatusName(TrilatStatus status)
{
    switch (status) {
        case TrilatStatus::Ok:
            return "OK";
        case TrilatStatus::TooFewRanges:
            return "TOO_FEW";
        case TrilatStatus::BadGeometry:
            return "GEOMETRY";
        case TrilatStatus::NonFinite:
            return "NON_FINITE";
    }
    return "UNKNOWN";
}

// 2x2 の対称行列 [a b; b c] の連立方程式を解く。配置が悪ければ false。
static bool trilatSolve2x2(float a, float b, float c, float rx, float ry, float& dx, float& dy)
{
    const float det   = (a * c) - (b * b);
    const float trace = a + c;
    if (!(trace > 0.0f)) return false;
    const float halfTrace = trace * 0.5f;
    if (!(det > (TRILAT_MIN_GEOMETRY * halfTrace * halfTrace))) return false;

    dx = ((c * rx) - (b * ry)) / det;
    dy = ((a * ry) - (b * rx)) / det;
    return true;
}

// 線形化した最小二乗で初期値を作る。座標は重心を原点にずらしたもの。
//
// 水平距離 h_i は測距値と高さの差から sqrt(r_i^2 - dz_i^2) で求める。雑音で r_i が
// dz_i を下回ったときは 0 にする (タグがアンカーの真下にいる扱い)。
static bool trilatLinearInit(const float* ax, const float* ay, const float* h2, size_t n, float& x, float& y)
{
    // 先頭のアンカーを基準にした i 番目の式:
    //   2 (x_i - x_0) x + 2 (y_i - y_0) y = h_0^2 - h_i^2 + (x_i^2 + y_i^2) - (x_0^2 + y_0^2)
    float a = 0.0f, b = 0.0f, c = 0.0f, rx = 0.0f, ry = 0.0f;
    const float k0 = (ax[0] * ax[0]) + (ay[0] * ay[0]);
    for (size_t i = 1; i < n; ++i) {
        const float gx = 2.0f * (ax[i] - ax[0]);
        const float gy = 2.0f * (ay[i] - ay[0]);
        const float v  = (h2[0] - h2[i]) + ((ax[i] * ax[i]) + (ay[i] * ay[i])) - k0;
        a += gx * gx;
        b += gx * gy;
        c += gy * gy;
        rx += gx * v;
        ry += gy * v;
    }
    return trilatSolve2x2(a, b, c, rx, ry, x, y);
}

// 残差 (推定位置からの距離 - 測距値) の 2 乗和を返す。
static float trilatResidualSq(const TrilatInput* inputs, const float* ax, const float* ay, size_t n, float tagZ,
                              float x, float y)
{
    float sum = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const float dx = x - ax[i];
        const float dy = y - ay[i];
        const float dz = tagZ - inputs[i].z;
        const float r  = sqrtf((dx * dx) + (dy * dy) + (dz * dz)) - inputs[i].range;
        sum += r * r;
    }
    return sum;
}

// inputs の n 本の測距からタグの水平位置を求める。tagZ はタグの高さ (メートル)。
static TrilatResult trilaterate2d(const TrilatInput* inputs, size_t n, float tagZ)
{
    TrilatResult result = {};
    result.status       = TrilatStatus::TooFewRanges;
    if (n > TRILAT_ANCHOR_MAX) n = TRILAT_ANCHOR_MAX;
    result.used = static_cast<uint8_t>(n);
    if (n < TRILAT_MIN_RANGES) return result;

    // 重心を原点にずらす。float で 2 乗の差を取るときの桁落ちを抑える。
    float cx = 0.0f, cy = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        cx += inputs[i].x;
        cy += inputs[i].y;
    }
    cx /= static_cast<float>(n);
    cy /= static_cast<float>(n);

    float ax[TRILAT_ANCHOR_MAX];
    float ay[TRILAT_ANCHOR_MAX];
    float h2[TRILAT_ANCHOR_MAX];
    for (size_t i = 0; i < n; ++i) {
        ax[i]          = inputs[i].x - cx;
        ay[i]          = inputs[i].y - cy;
        const float dz = tagZ - inputs[i].z;
        const float r  = inputs[i].range;
        const float v  = (r * r) - (dz * dz);
        h2[i]          = (v > 0.0f) ? v : 0.0f;
    }

    // 線形化した解が作れないとき (アンカーの水平位置が一直線に並ぶなど) は重心から
    // 始める。その場合でも Gauss-Newton 側の配置の判定で弾かれることが多い。
    float x = 0.0f, y = 0.0f;
    if (!trilatLinearInit(ax, ay, h2, n, x, y)) {
        x = 0.0f;
        y = 0.0f;
    }

    for (int iter = 0; iter < TRILAT_MAX_ITERATIONS; ++iter) {
        // 正規方程式 (J^T J) d = -J^T r。J の各行は推定位置からアンカーへの
        // 単位ベクトルの水平成分で、高さの差が大きいほど短くなる。
        float a = 0.0f, b = 0.0f, c = 0.0f, gx = 0.0f, gy = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            const float dx  = x - ax[i];
            const float dy  = y - ay[i];
            const float dz  = tagZ - inputs[i].z;
            const float rho = sqrtf((dx * dx) + (dy * dy) + (dz * dz));
            // タグとアンカーが同じ位置に重なった場合。方向が決まらないので
            // この式は勾配に寄与させない。
            if (!(rho > 1e-6f)) continue;
            const float jx = dx / rho;
            const float jy = dy / rho;
            const float r  = rho - inputs[i].range;
            a += jx * jx;
            b += jx * jy;
            c += jy * jy;
            gx += jx * r;
            gy += jy * r;
        }

        float stepX = 0.0f, stepY = 0.0f;
        if (!trilatSolve2x2(a, b, c, -gx, -gy, stepX, stepY)) {
            result.status = TrilatStatus::BadGeometry;
            return result;
        }
        x += stepX;
        y += stepY;
        result.iterations = static_cast<uint8_t>(iter + 1);
        if (!std::isfinite(x) || !std::isfinite(y)) {
            result.status = TrilatStatus::NonFinite;
            return result;
        }
        if (((stepX * stepX) + (stepY * stepY)) < (TRILAT_CONVERGED_M * TRILAT_CONVERGED_M)) {
            result.converged = true;
            break;
        }
    }

    const float sumSq = trilatResidualSq(inputs, ax, ay, n, tagZ, x, y);
    // 収束しなかった場合も、有限の値が出ていれば解として返す。発散の様子は
    // 残差と一緒にテレメトリで見たいので、ここで握り潰さない (doc/server-design.md 6.2)。
    result.status      = TrilatStatus::Ok;
    result.x           = x + cx;
    result.y           = y + cy;
    result.residualRms = sqrtf(sumSq / static_cast<float>(n));
    if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.residualRms)) {
        result.status = TrilatStatus::NonFinite;
    }
    return result;
}

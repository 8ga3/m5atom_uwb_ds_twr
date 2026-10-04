// 測距値を直接の観測にする拡張カルマンフィルタ (EKF) による 2D の位置推定。
//
// 巡回の 1 周期ごとに 3.6 の最小二乗 (trilateration.h) で位置を求めるのとは別に、
// 測距 1 本ずつを観測としてフィルタへ取り込み、走行中の位置を滑らかにする
// (doc/multi-anchor-positioning-design.md 3.7)。
//
// - 状態は水平位置と速度 (x, y, vx, vy) の 4 つ。運動モデルは等速度で、加速度を
//   白色雑音としてプロセスノイズに入れる
// - 観測は 1 本の測距 (斜距離) で、タグの高さは最小二乗と同じ固定値で与える
// - 測距は巡回のスロットごとに時刻がずれるので、取り込む前にその測距の時刻まで予測を進める
// - 観測の残差 (イノベーション) が予測の不確かさに比べて大きすぎる測距は棄却する
// - 初期値は最小二乗の解から作る。最小二乗が解けなくても、測距が 1 本でもあれば更新できる
//
// 予測だけでつなぐのは、最後に測距を取り込んでからの時間と位置の標準偏差の両方に
// 上限を設けてその範囲までとし、超えたらフィルタ後の位置を無効にする。無効にした後は、
// 次に最小二乗が解けて、その残差 RMS が PFILTER_INIT_RESIDUAL_MAX_M 以下だった周期で
// 初期化し直す。残差が大きい解は外れた測距を含む疑いがあるので使わず、その間は無効のままにする。
//
// trilateration.h と同じく Arduino にも M5 のライブラリにも依存させず、ホストの
// C++ コンパイラでそのまま試験できるようにする。計算は float で行う。
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

// プロセスノイズ。加速度を白色雑音とみなしたときのパワースペクトル密度 (m^2/s^3)。
// 1 秒あたりで速度の標準偏差が sqrt(この値) m/s だけ広がる。ローバーの加減速
// (1 m/s^2 程度) を見込んで 1 とする。大きくすると追従が速く、小さくすると滑らかになる。
// 実機で比べるため、ビルドフラグ -D UWB_KF_ACCEL_PSD=<値> でソースコードを書き換えずに変えられるようにする
// (値を変えたらビルドし直して書き込む)
// (doc/multi-anchor-positioning-design.md 3.7)。
#ifndef UWB_KF_ACCEL_PSD
#define UWB_KF_ACCEL_PSD 1.0
#endif
static_assert((UWB_KF_ACCEL_PSD) > 0, "UWB_KF_ACCEL_PSD must be positive");
static constexpr float PFILTER_ACCEL_PSD = static_cast<float>(UWB_KF_ACCEL_PSD);

// 観測ノイズ (測距 1 本の標準偏差、メートル)。静止時の測距のばらつきは 10〜20 mm
// だが、アンカーごとに 18〜70 mm の補正しきれないずれがあるので (3.6)、それを含めて 50 mm とする。
static constexpr float PFILTER_RANGE_SIGMA_M = 0.05f;

// イノベーションによる棄却のしきい値。イノベーションの 2 乗をその分散で割った値
// (正規化したイノベーションの 2 乗) がこの値を超えた測距は取り込まない。
// 16 は標準偏差の 4 倍にあたる。3.6 の負の外れ値 (-3.2 m 以下) は確実に弾ける。
static constexpr float PFILTER_GATE_NIS = 16.0f;

// 初期化に使う最小二乗の解の残差 RMS の上限 (メートル)。静止時の残差は 20〜50 mm。
// これより大きい解は外れた測距を含んでいる疑いがあるので、初期値には使わない。
static constexpr float PFILTER_INIT_RESIDUAL_MAX_M = 0.15f;

// 初期化したときの位置と速度の標準偏差。速度は分からないので 0 とし、走行中に
// 初期化しても追い付けるよう 1 m/s の幅を持たせる。
static constexpr float PFILTER_INIT_POS_SIGMA_M   = 0.1f;
static constexpr float PFILTER_INIT_VEL_SIGMA_MPS = 1.0f;

// 予測だけでつなげる上限。最後に測距を取り込んでからこの時間を超えるか、位置の
// 標準偏差 sqrt(Pxx + Pyy) がこの値を超えたら、フィルタ後の位置を無効にする。
// 1 秒は 10Hz で 10 周期、30Hz で 30 周期にあたる。
static constexpr uint32_t PFILTER_COAST_MAX_MS = 1000;
static constexpr float PFILTER_POS_SIGMA_MAX_M = 0.5f;

// 推定位置とアンカーの水平距離がこれ以下の測距は取り込まない (メートル)。真下にいるときの測距は
// 水平位置の情報を持たないためで、1 mm は観測行列の水平成分がほぼ 0 になる範囲にあたる。
static constexpr float PFILTER_MIN_HORIZONTAL_M = 0.001f;

struct PositionFilterRange {
    float x;        // アンカーの設置座標 (メートル)
    float y;
    float z;
    float range;    // 測距値 (メートル。バイアス補正済み)
    uint32_t tMs;   // その測距を始めた millis()
};

// フィルタ後の位置を無効にした理由。シリアルの統計で数える。
enum class PositionFilterReset : uint8_t {
    None,
    CoastTimeout,  // 測距を取り込めない時間が PFILTER_COAST_MAX_MS を超えた
    SigmaLimit,    // 位置の標準偏差が PFILTER_POS_SIGMA_MAX_M を超えた
    NonFinite,     // 計算の途中で NaN / 無限大が出た
};

// 1 周期ぶんの処理の結果。
struct PositionFilterCycle {
    bool valid;         // 周期の終わりでフィルタ後の位置が有効か
    bool updated;       // この周期で観測 (測距、または初期化に使った最小二乗の解) を取り込んだか
    bool initialized;   // この周期で最小二乗の解から初期化したか
    uint8_t used;       // 取り込んだ測距の本数 (初期化した周期は最小二乗に使った本数)
    uint8_t rejected;   // イノベーションの大きさで棄却した測距の本数
    PositionFilterReset reset;  // この周期で無効にした理由。無効にしなければ None
    float x;            // 周期の終わり (最後のスロットの時刻) での位置と速度。valid のときだけ意味を持つ
    float y;
    float vx;
    float vy;
    float sigma;        // 位置の標準偏差 sqrt(Pxx + Pyy) (メートル)
};

static const char* positionFilterResetName(PositionFilterReset reset)
{
    switch (reset) {
        case PositionFilterReset::None:
            return "NONE";
        case PositionFilterReset::CoastTimeout:
            return "COAST";
        case PositionFilterReset::SigmaLimit:
            return "SIGMA";
        case PositionFilterReset::NonFinite:
            return "NON_FINITE";
    }
    return "UNKNOWN";
}

class PositionFilter {
public:
    // フィルタ後の位置を無効にする。構成を入れ替えたときにも呼ぶ。
    void reset()
    {
        valid_ = false;
    }

    bool valid() const
    {
        return valid_;
    }

    // 1 周期ぶんの測距を取り込む。ranges は時刻順 (巡回の順) に並べ、使えない測距
    // (失敗、有り得ない負の値) は除いておく。endMs はその周期の最後のスロットの時刻で、
    // 結果の位置はこの時刻まで予測を進めたものになる。
    //
    // lsOk / lsX / lsY / lsResidual / lsUsed は同じ周期の最小二乗の結果。フィルタが無効な
    // ときの初期化にだけ使う。初期化した周期の測距は、最小二乗の解に使ったものと同じなので
    // フィルタへは取り込まない (同じ観測を 2 回使うことになる)。
    PositionFilterCycle step(const PositionFilterRange* ranges, size_t n, float tagZ, uint32_t endMs, bool lsOk,
                             float lsX, float lsY, float lsResidual, uint8_t lsUsed)
    {
        PositionFilterCycle out = {};
        out.reset               = PositionFilterReset::None;

        if (valid_) {
            for (size_t i = 0; i < n; ++i) {
                predictTo(ranges[i].tMs);
                switch (update(ranges[i], tagZ)) {
                    case UpdateResult::Accepted:
                        ++out.used;
                        lastUpdateMs_ = ranges[i].tMs;
                        break;
                    case UpdateResult::Rejected:
                        ++out.rejected;
                        break;
                    case UpdateResult::Skipped:
                        break;
                }
            }
            predictTo(endMs);
            out.updated = (out.used > 0);

            const float sigma = positionSigma();
            if (!stateFinite() || !std::isfinite(sigma)) {
                out.reset = PositionFilterReset::NonFinite;
            } else if ((endMs - lastUpdateMs_) > PFILTER_COAST_MAX_MS) {
                out.reset = PositionFilterReset::CoastTimeout;
            } else if (sigma > PFILTER_POS_SIGMA_MAX_M) {
                out.reset = PositionFilterReset::SigmaLimit;
            }
            if (out.reset != PositionFilterReset::None) {
                valid_      = false;
                out.updated = false;
            }
        }

        // 無効なら、同じ周期の最小二乗の解で初期化する。無効にした直後の周期でも
        // 最小二乗が解けていればすぐに戻す。測距をすべて棄却し続けて無効になった場合は、
        // フィルタの状態の方が外れていたとみなす。used は初期化に使った最小二乗の本数に
        // 置き換え、rejected は無効にする前に棄却した本数をそのまま残す。
        if (!valid_ && lsOk && std::isfinite(lsX) && std::isfinite(lsY)
            && (lsResidual <= PFILTER_INIT_RESIDUAL_MAX_M)) {
            initialize(lsX, lsY, endMs);
            out.updated     = true;
            out.initialized = true;
            out.used        = lsUsed;
        }

        out.valid = valid_;
        if (valid_) {
            out.x     = x_[0];
            out.y     = x_[1];
            out.vx    = x_[2];
            out.vy    = x_[3];
            out.sigma = positionSigma();
        }
        return out;
    }

private:
    enum class UpdateResult : uint8_t { Accepted, Rejected, Skipped };

    void initialize(float x, float y, uint32_t tMs)
    {
        x_[0] = x;
        x_[1] = y;
        x_[2] = 0.0f;
        x_[3] = 0.0f;
        for (auto& row : p_) {
            for (float& v : row) v = 0.0f;
        }
        const float pos2 = PFILTER_INIT_POS_SIGMA_M * PFILTER_INIT_POS_SIGMA_M;
        const float vel2 = PFILTER_INIT_VEL_SIGMA_MPS * PFILTER_INIT_VEL_SIGMA_MPS;
        p_[0][0]         = pos2;
        p_[1][1]         = pos2;
        p_[2][2]         = vel2;
        p_[3][3]         = vel2;
        tMs_             = tMs;
        lastUpdateMs_    = tMs;
        valid_           = true;
    }

    // 等速度モデルで tMs まで予測を進める。時刻が戻る (順序が入れ替わった) 場合は進めない。
    void predictTo(uint32_t tMs)
    {
        const int32_t deltaMs = static_cast<int32_t>(tMs - tMs_);
        if (deltaMs <= 0) return;
        tMs_           = tMs;
        const float dt = static_cast<float>(deltaMs) * 0.001f;

        x_[0] += x_[2] * dt;
        x_[1] += x_[3] * dt;

        // P = F P F^T + Q。F は単位行列の (0, 2) と (1, 3) に dt を入れたもので、位置の行
        // (列) に速度の行 (列) の dt 倍を足す操作になる。行列の積を一般の形で回さずに書き下す。
        float f[4][4];
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                // F P: 位置の行に速度の行 x dt を足す
                f[r][c] = p_[r][c] + ((r < 2) ? (p_[r + 2][c] * dt) : 0.0f);
            }
        }
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                // (F P) F^T: 位置の列に速度の列 x dt を足す
                p_[r][c] = f[r][c] + ((c < 2) ? (f[r][c + 2] * dt) : 0.0f);
            }
        }

        // 加速度を白色雑音としたときの離散化したプロセスノイズ。
        //   Q = q [dt^3/3, dt^2/2; dt^2/2, dt] を x と y の軸それぞれに置く
        const float q   = PFILTER_ACCEL_PSD;
        const float dt2 = dt * dt;
        const float qpp = q * dt2 * dt / 3.0f;
        const float qpv = q * dt2 / 2.0f;
        const float qvv = q * dt;
        for (int i = 0; i < 2; ++i) {
            p_[i][i] += qpp;
            p_[i][i + 2] += qpv;
            p_[i + 2][i] += qpv;
            p_[i + 2][i + 2] += qvv;
        }
    }

    // 測距 1 本で更新する。観測は 1 次元なので、カルマンゲインの計算に逆行列は要らない。
    UpdateResult update(const PositionFilterRange& range, float tagZ)
    {
        const float dx  = x_[0] - range.x;
        const float dy  = x_[1] - range.y;
        const float dz  = tagZ - range.z;
        const float rho = sqrtf((dx * dx) + (dy * dy) + (dz * dz));
        // タグがアンカーの真下 (真上) にいると、測距は水平位置について何も教えない (観測行列の水平成分が 0)。
        // 状態も共分散も変わらないのに取り込んだと数えると、取り込めない時間の判定 (PFILTER_COAST_MAX_MS) が
        // 遅れるので使わない。タグとアンカーが同じ位置に重なった場合 (rho が 0) もここで除かれる。
        const float horizontal = sqrtf((dx * dx) + (dy * dy));
        if (!(horizontal > PFILTER_MIN_HORIZONTAL_M)) return UpdateResult::Skipped;

        // 観測行列 H = [dx/rho, dy/rho, 0, 0]。速度には直接かからない。
        const float hx = dx / rho;
        const float hy = dy / rho;

        // P H^T と、イノベーションの分散 S = H P H^T + R
        float ph[4];
        for (int r = 0; r < 4; ++r) ph[r] = (p_[r][0] * hx) + (p_[r][1] * hy);
        const float s = (hx * ph[0]) + (hy * ph[1]) + (PFILTER_RANGE_SIGMA_M * PFILTER_RANGE_SIGMA_M);
        if (!(s > 0.0f)) return UpdateResult::Skipped;

        const float innovation = range.range - rho;
        if (!((innovation * innovation) <= (PFILTER_GATE_NIS * s))) return UpdateResult::Rejected;

        // K = P H^T / S、x += K ν、P -= K (P H^T)^T。P H^T の外積を S で割る形なので P は対称のまま。
        for (int r = 0; r < 4; ++r) x_[r] += (ph[r] / s) * innovation;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) p_[r][c] -= (ph[r] * ph[c]) / s;
        }
        return UpdateResult::Accepted;
    }

    float positionSigma() const
    {
        // 共分散が壊れて (NaN や負の値になって) いたら、0 に丸めずに NaN のまま返す。step() の
        // std::isfinite() の判定で NonFinite として無効にするためである。
        return sqrtf(p_[0][0] + p_[1][1]);
    }

    bool stateFinite() const
    {
        for (float v : x_) {
            if (!std::isfinite(v)) return false;
        }
        return true;
    }

    bool valid_            = false;
    float x_[4]            = {};  // x, y, vx, vy
    float p_[4][4]         = {};
    uint32_t tMs_          = 0;   // 状態の時刻 (millis())
    uint32_t lastUpdateMs_ = 0;   // 最後に観測を取り込んだ時刻
};

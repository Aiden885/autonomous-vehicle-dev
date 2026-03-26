#include "calcPreviewDistByCurvature.hpp"
#include <algorithm>
#include <cmath>

double calcPreviewDistByCurvature(
    const std::vector<double>& kappaList,
    int    previewIdx,
    double dT,

    double k_min,
    double k_max,
    double minPreviewDist,
    double maxPreviewDist,

    int    winN,
    double k_enter,
    double k_exit,

    double tau_L,
    double dL_upMax,
    double dL_downMax,
    double initPreviewDist)
{
    // ================== 内部状态 ==================
    static bool   in_curve           = false;
    static double previewDist_prev   = initPreviewDist;
    static double previewDist_lp     = initPreviewDist;

    // ================== 1. 曲率插值映射 ==================
    const double k = kappaList[previewIdx];
    double desiredDist;

    if (k <= k_min) {
        desiredDist = maxPreviewDist;
    } else if (k >= k_max) {
        desiredDist = minPreviewDist;
    } else {
        const double ratio = (k_max - k) / (k_max - k_min);
        desiredDist = minPreviewDist +
                      ratio * (maxPreviewDist - minPreviewDist);
    }

    // ================== 2. 弯道进入/退出判定 ==================
    int endIdx = std::min<int>(previewIdx + winN,
                               static_cast<int>(kappaList.size()) - 1);

    double k_win_max = 0.0;
    for (int i = previewIdx; i <= endIdx; ++i) {
        k_win_max = std::max(k_win_max, std::fabs(kappaList[i]));
    }

    if (!in_curve && k_win_max > k_enter) in_curve = true;
    if ( in_curve && k_win_max < k_exit ) in_curve = false;

    // ================== 3. 一阶低通 ==================
    const double a = 1.0 - std::exp(-dT / tau_L);
    previewDist_lp = previewDist_lp + a * (desiredDist - previewDist_lp);

    // ================== 4. 变率限制 ==================
    const bool lengthen = (previewDist_lp > previewDist_prev);

    const double dL_down_max = in_curve ? dL_downMax : 5.8;
    const double dL_up_max   = in_curve ? 2.8       : dL_upMax;

    const double dL    = previewDist_lp - previewDist_prev;
    const double limit = (lengthen ? dL_up_max : dL_down_max) * dT;

    double previewDist_new;
    if (lengthen) {
        previewDist_new = (dL >  limit)
                            ? (previewDist_prev + limit)
                            : previewDist_lp;
    } else {
        previewDist_new = (dL < -limit)
                            ? (previewDist_prev - limit)
                            : previewDist_lp;
    }

    // ================== 5. 限幅 + 状态更新 ==================
    const double previewDist =
        std::max(minPreviewDist,
                 std::min(maxPreviewDist, previewDist_new));

    previewDist_prev = previewDist;
    return previewDist;
}

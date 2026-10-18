// RDK X5 person-only YOLOv8 DFL decode + NMS.
// Pure C++ (no OpenCV), called from Python via ctypes.
//
// Inputs are NHWC float32 raw logits:
//   cls: [n,n,1]   box: [n,n,64]   for n = size/8, size/16, size/32
// Output: boxes xyxy in original image pixels + scores, sorted by score.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

extern "C" {

struct Det {
    float x1, y1, x2, y2, s;
};

static inline float iou(const Det &a, const Det &b) {
    float xx1 = std::max(a.x1, b.x1);
    float yy1 = std::max(a.y1, b.y1);
    float xx2 = std::min(a.x2, b.x2);
    float yy2 = std::min(a.y2, b.y2);
    float w = std::max(0.f, xx2 - xx1);
    float h = std::max(0.f, yy2 - yy1);
    float inter = w * h;
    float uni = (a.x2 - a.x1) * (a.y2 - a.y1)
              + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
    return inter / (uni + 1e-9f);
}

int yolo_person_post(const float *c8, const float *b8,
                     const float *c4, const float *b4,
                     const float *c2, const float *b2,
                     int size, float conf, float iou_thr,
                     float ratio, float padx, float pady,
                     int ow, int oh,
                     float *out_boxes, float *out_scores, int max_det) {
    const float thr = std::log(conf / (1.f - conf));
    const float *cs[3] = {c8, c4, c2};
    const float *bs[3] = {b8, b4, b2};
    const int strides[3] = {8, 16, 32};

    std::vector<Det> dets;
    dets.reserve(256);

    for (int k = 0; k < 3; ++k) {
        const int s = strides[k];
        const int n = size / s;
        const float *c = cs[k];
        const float *b = bs[k];
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const int idx = y * n + x;
                const float lg = c[idx];
                if (lg < thr) continue;

                const float *p = b + (size_t)idx * 64;
                float d[4];
                for (int e = 0; e < 4; ++e) {
                    const float *q = p + e * 16;
                    float m = q[0];
                    for (int i = 1; i < 16; ++i) m = std::max(m, q[i]);
                    float sum = 0.f, acc = 0.f;
                    for (int i = 0; i < 16; ++i) {
                        float v = std::exp(q[i] - m);
                        sum += v;
                        acc += v * (float)i;
                    }
                    d[e] = acc / sum;
                }

                const float cx = (x + 0.5f) * s;
                const float cy = (y + 0.5f) * s;
                float x1 = (cx - d[0] * s - padx) / ratio;
                float y1 = (cy - d[1] * s - pady) / ratio;
                float x2 = (cx + d[2] * s - padx) / ratio;
                float y2 = (cy + d[3] * s - pady) / ratio;

                x1 = std::max(0.f, std::min(x1, (float)ow));
                y1 = std::max(0.f, std::min(y1, (float)oh));
                x2 = std::max(0.f, std::min(x2, (float)ow));
                y2 = std::max(0.f, std::min(y2, (float)oh));
                if (x2 <= x1 || y2 <= y1) continue;

                dets.push_back({x1, y1, x2, y2, 1.f / (1.f + std::exp(-lg))});
            }
        }
    }

    std::sort(dets.begin(), dets.end(),
              [](const Det &a, const Det &b) { return a.s > b.s; });

    std::vector<int> keep;
    keep.reserve(max_det);
    for (int i = 0; i < (int)dets.size() && (int)keep.size() < max_det; ++i) {
        bool ok = true;
        for (int j : keep) {
            if (iou(dets[i], dets[j]) > iou_thr) { ok = false; break; }
        }
        if (ok) keep.push_back(i);
    }

    const int n = (int)keep.size();
    for (int i = 0; i < n; ++i) {
        const Det &d = dets[keep[i]];
        out_boxes[4 * i + 0] = d.x1;
        out_boxes[4 * i + 1] = d.y1;
        out_boxes[4 * i + 2] = d.x2;
        out_boxes[4 * i + 3] = d.y2;
        out_scores[i] = d.s;
    }
    return n;
}

}  // extern "C"

#ifndef FILTER_H
#define FILTER_H
typedef struct { float alpha, y; int init; } lp_t;

static inline void lp_init(lp_t* f, float alpha) { f->alpha = alpha; f->y = 0; f->init = 0; }
static inline float lp_apply(lp_t* f, float x) {
    if (!f->init) { f->y = x; f->init = 1; return f->y; }
    f->y = f->alpha * x + (1.0f - f->alpha) * f->y;
    return f->y;
}
#endif

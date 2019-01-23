#ifndef H_COLORS
#define H_COLORS

typedef struct {
    double r;       // [0,1], truncated automatically
    double g;       // [0,1], truncated automatically
    double b;       // [0,1], truncated automatically
} rgbColor;

typedef struct {
    double h;       // [0, 360); taken mod 360
    double s;       // [0,1], truncated automatically
    double v;       // [0,1], truncated automatically
} hsvColor;

hsvColor rgb2hsv(rgbColor in);
rgbColor hsv2rgb(hsvColor in);

#endif

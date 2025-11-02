#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <chrono>
#include <cstring>

// (Pusula fonksiyonlarını olduğu gibi tutuyoruz; sadece çağrıyı yorumlayacağız)

// RGB565 dönüşümü
static inline uint16_t BGR2RGB565(uint8_t b, uint8_t g, uint8_t r) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

int main() {
    // --- Kamera: 128x128 native, BGR ---
    cv::VideoCapture cap(
        "libcamerasrc ! videoconvert ! "
        "video/x-raw,width=128,height=128,format=BGR ! appsink",
        cv::CAP_GSTREAMER);
    if (!cap.isOpened()) return -1;

    // --- Framebuffer: OLED /dev/fb1 ---
    int fb = open("/dev/fb1", O_RDWR);
    if (fb < 0) { perror("fb1 açılamadı"); return -1; }
    fb_var_screeninfo vinfo;
    if (ioctl(fb, FBIOGET_VSCREENINFO, &vinfo) < 0) { perror("FBIOGET_VSCREENINFO"); return -1; }
    size_t screensize = (size_t)vinfo.yres_virtual * (size_t)vinfo.xres_virtual * vinfo.bits_per_pixel / 8;
    uint16_t* fbp = (uint16_t*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
    if (fbp == MAP_FAILED) { perror("mmap başarısız"); return -1; }

    // --- Gamma LUT (gamma = 0.45) ---
    cv::Mat lut(1, 256, CV_8U);
    for (int i = 0; i < 256; ++i) {
        float n = i / 255.f;
        float g = std::pow(n, 0.45f);
        lut.at<uchar>(i) = (uchar)std::round(std::min(255.f, std::max(0.f, g * 255.f)));
    }

    cv::Mat frame, gray, grayGamma, greenPhos;
    // float heading = 0;  // pusula şimdilik devre dışı

    while (true) {
        auto t0 = std::chrono::high_resolution_clock::now();

        cap >> frame;
        if (frame.empty()) break;              // 128x128 BGR

        // NVG pipeline: gri → gamma LUT → fosfor
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::LUT(gray, lut, grayGamma);
        cv::applyColorMap(grayGamma, greenPhos, cv::COLORMAP_SUMMER);

        // (Pusula kapalı)
        // heading += -1;
        // drawTopCompass(greenPhos, heading);

        // OLED'e yaz (RGB565)
        int maxY = std::min(greenPhos.rows, (int)vinfo.yres);
        int maxX = std::min(greenPhos.cols, (int)vinfo.xres);
        for (int y = 0; y < maxY; ++y) {
            uint16_t* dst = fbp + y * vinfo.xres;
            const cv::Vec3b* src = greenPhos.ptr<cv::Vec3b>(y);
            for (int x = 0; x < maxX; ++x) {
                const cv::Vec3b& bgr = src[x];
                dst[x] = BGR2RGB565(bgr[0], bgr[1], bgr[2]);
            }
        }

        // ~60 FPS cap
        auto t1 = std::chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        if (ms < 16) usleep((16 - ms) * 1000);
    }

    if (fbp && fbp != MAP_FAILED) munmap(fbp, screensize);
    if (fb >= 0) close(fb);
    cap.release();
    return 0;
}

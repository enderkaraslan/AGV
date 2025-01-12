#pragma once

#include <opencv2/opencv.hpp>
#include "Constants.h"

namespace AGV
{
class ImageProcessor {
public:
    struct ContourAnalysisResult {
        cv::Point contour_center    = {0, 0};              // Kontur merkez noktası
        double extent               = 0.0;                 // Kontur oranı (extent)
        double area                 = 0.0;                 // Kontur alanı
        int left_black_pixel_count  = 0;                   // Sol bölgedeki siyah piksel sayısı
        int right_black_pixel_count = 0;                   // Sağ bölgedeki siyah piksel sayısı
        int mid_pixel               = 0;                   // Orta noktadaki piksel değeri
        bool valid                  = false;               // Konturun geçerli olup olmadığını belirtir
        int width                   = 0;                   // Resim genişliği
        int height                  = 0;                   // Resim yüksekliği
        int middle_x                = 0;                   // Resmin orta x koordinatı
        int middle_y                = 0;                   // Resmin orta y koordinatı
    };

    static ContourAnalysisResult process(const cv::Mat &image);  // Görüntüyü işleyen ana metot

private:
    static cv::Mat preprocessImage(const cv::Mat &image);                                               // Görüntüyü işlemek için ön işlemler
    static std::vector<cv::Point> getMainContour(const std::vector<std::vector<cv::Point>> &contours);  // En büyük konturu bulur
    static cv::Mat getPart(const cv::Mat &image, int y_start, int y_end, int x_start, int x_end);       // Görüntünün belirli bir bölümünü alır
    static cv::Point getContourCenter(const std::vector<cv::Point> &contour);                           // Konturun merkezini hesaplar
    static double getContourExtent(const std::vector<cv::Point> &contour);                              // Konturun oranını hesaplar

};  // class ImageProcessor
}   // namespace AGV

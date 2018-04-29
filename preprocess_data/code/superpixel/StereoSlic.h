#ifndef STEREO_SLIC_H
#define STEREO_SLIC_H

#include <vector>
#include <png++/png.hpp>

class StereoSlic {
public:
    StereoSlic();
    
    void setIterationTotal(const int iterationTotal);
    void setEnergyParameter(const double colorWeight, const double disparityWeight, const double noDisparityPenalty);

    void segment(const int superpixelTotal,
                 const png::image<png::rgb_pixel>& leftImage,
                 png::image<png::gray_pixel_16>& segmentImage);
    void segment(const int superpixelTotal,
                 const png::image<png::rgb_pixel>& leftImage,
                 const png::image<png::gray_pixel_16>& leftDisparityImage,
                 png::image<png::gray_pixel_16>& segmentImage,
                 png::image<png::gray_pixel_16>& disparityImage);
    void segment(const int superpixelTotal,
                 const png::image<png::rgb_pixel>& leftImage,
                 const png::image<png::gray_pixel_16>& leftDisparityImage,
                 const png::image<png::gray_pixel_16>& rightDisparityImage,
                 png::image<png::gray_pixel_16>& segmentImage,
                 png::image<png::gray_pixel_16>& disparityImage);
    void segment(const int superpixelTotal,
                 const png::image<png::rgb_pixel>& leftImage,
                 const png::image<png::gray_pixel_16>& leftDisparityImage,
                 const png::image<png::gray_pixel_16>& rightDisparityImage,
                 const png::image<png::rgb_pixel>& rightImage,
                 png::image<png::gray_pixel_16>& segmentImage,
                 png::image<png::gray_pixel_16>& disparityImage);
    
private:
    struct LabXYD {
        LabXYD() {
            color[0] = 0;  color[1] = 0;  color[2] = 0;
            position[0] = 0;  position[1] = 0;
            disparityPlane[0] = 0;  disparityPlane[1] = 0;  disparityPlane[2] = -1;
        }
        
        double color[3];
        double position[2];
        double disparityPlane[3];
    };
    struct DisparityPixel {
        double x;
        double y;
        double d;
    };
    
    void setColorImage(const png::image<png::rgb_pixel>& leftImage);
    void setColorImage(const png::image<png::rgb_pixel>& leftImage,
                       const png::image<png::rgb_pixel>& rightImage);
    void setDisparityImage(const png::image<png::gray_pixel_16>& leftDisparityImage);
    void setDisparityImage(const png::image<png::gray_pixel_16>& leftDisparityImage,
                           const png::image<png::gray_pixel_16>& rightDisparityImage);
    void performSegmentation(const int superpixelTotal);
    void makeSegmentImage(png::image<png::gray_pixel_16>& segmentImage) const;
    void makeDisparityImage(png::image<png::gray_pixel_16>& disparityImage);
    
    void convertRGBToLab(const png::image<png::rgb_pixel>& rgbImage,
                         std::vector<float>& labImage);
    void calcLeftLabEdges();
    void initializeSeeds(const int superpixelTotal);
    void assignLabel();
    void updateSeeds();
    void updateSeedsColorAndPosition();
    void estimateDisparityPlaneParameter();
    std::vector< std::vector<DisparityPixel> > makeDisparityPixelList() const;
    std::vector<double> estimateDisparityPlaneParameter(const std::vector<DisparityPixel>& disparityPixels) const;
    std::vector<double> estimateDisparityPlaneParameterRANSAC(const std::vector<DisparityPixel>& disparityPixels) const;
    void enforceLabelConnectivity();
    void labelConnectedPixels(const int x,
                              const int y,
                              const int newLabelIndex,
                              std::vector<int>& newLabels,
                              std::vector<int>& connectedXs,
                              std::vector<int>& connectedYs) const;
    
    
    // Parameter
    int energyType_;
    int iterationTotal_;
    double gridSize_;
    double colorWeight_;
    double disparityWeight_;
    double noDisparityPenalty_;
    
    // Data
    std::vector<int> labels_;
    
    // Color and disparity images
    int width_;
    int height_;
    std::vector<float> leftLabImage_;
    std::vector<float> leftDisparityImage_;
    std::vector<float> rightLabImage_;
    std::vector<float> rightDisparityImage_;
    std::vector<double> leftLabEdges_;
    
    // Superpixel segments
    std::vector<LabXYD> seeds_;
    int stepSize_;
};

png::image<png::rgb_pixel> drawSegmentBoundary(const png::image<png::rgb_pixel>& originalImage,
                                               const png::image<png::gray_pixel_16>& segmentImage,
                                               const png::rgb_pixel boundaryColor = png::rgb_pixel(255, 0, 0));

#endif

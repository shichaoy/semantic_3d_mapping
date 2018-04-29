#include "StereoSlic.h"
#include <cmath>
#include <float.h>
#include <exception>
#include <Eigen/Dense>

// Default parameter
const int STEREOSLIC_DEFAULT_ITERATION_TOTAL = 10;
const double STEREOSLIC_DEFAULT_COLOR_WEIGHT = 3000.0;
const double STEREOSLIC_DEFAULT_DISPARITY_WEIGHT = 30.0;
const double STEREOSLIC_DEFAULT_NO_DISPARITY_PENALTY = 3.0;

// Pixel offsets of 4- and 8-neighbors
const int fourNeighborTotal = 4;
const int fourNeighborOffsetX[4] = {-1, 0, 1, 0};
const int fourNeighborOffsetY[4] = { 0,-1, 0, 1};
const int eightNeighborTotal = 8;
const int eightNeighborOffsetX[8] = {-1,-1, 0, 1, 1, 1, 0,-1};
const int eightNeighborOffsetY[8] = { 0,-1,-1,-1, 0, 1, 1, 1};

// Prototype declaration
int computeRequiredSamplingTotal(const int drawTotal, const int inlierTotal, const int pointTotal,
                                 const int currentSamplingTotal, const double confidenceLevel);


StereoSlic::StereoSlic() : iterationTotal_(STEREOSLIC_DEFAULT_ITERATION_TOTAL),
                           colorWeight_(STEREOSLIC_DEFAULT_COLOR_WEIGHT),
                           disparityWeight_(STEREOSLIC_DEFAULT_DISPARITY_WEIGHT),
                           noDisparityPenalty_(STEREOSLIC_DEFAULT_NO_DISPARITY_PENALTY) {}

void StereoSlic::setIterationTotal(const int iterationTotal) {
    if (iterationTotal < 1) {
        throw std::runtime_error("StereoSlic: the number of iterations is less than 1");
    }
    
    iterationTotal_ = iterationTotal;
}

void StereoSlic::setEnergyParameter(const double colorWeight, const double disparityWeight, const double noDisparityPenalty) {
    if (colorWeight <= 0 || disparityWeight < 0 || noDisparityPenalty < 0) {
        throw std::runtime_error("StereoSlic: energy parameter is less than zero");
    }
    
    colorWeight_ = colorWeight;
    disparityWeight_ = disparityWeight;
    noDisparityPenalty_ = noDisparityPenalty;
}

void StereoSlic::segment(const int superpixelTotal,
                         const png::image<png::rgb_pixel>& leftImage,
                         png::image<png::gray_pixel_16>& segmentImage)
{
    energyType_ = 0;   // Color(left)
    setColorImage(leftImage);
    performSegmentation(superpixelTotal);
    makeSegmentImage(segmentImage);
}

void StereoSlic::segment(const int superpixelTotal,
                         const png::image<png::rgb_pixel>& leftImage,
                         const png::image<png::gray_pixel_16>& leftDisparityImage,
                         png::image<png::gray_pixel_16>& segmentImage,
                         png::image<png::gray_pixel_16>& disparityImage)
{
    energyType_ = 1;   // Color(left) + Disparity(left)
    setColorImage(leftImage);
    setDisparityImage(leftDisparityImage);
    performSegmentation(superpixelTotal);
    makeSegmentImage(segmentImage);
    makeDisparityImage(disparityImage);
}

void StereoSlic::segment(const int superpixelTotal,
                         const png::image<png::rgb_pixel>& leftImage,
                         const png::image<png::gray_pixel_16>& leftDisparityImage,
                         const png::image<png::gray_pixel_16>& rightDisparityImage,
                         png::image<png::gray_pixel_16>& segmentImage,
                         png::image<png::gray_pixel_16>& disparityImage)
{
    energyType_ = 2;   // Color(left) + Disparity(left/right)
    setColorImage(leftImage);
    setDisparityImage(leftDisparityImage, rightDisparityImage);
    performSegmentation(superpixelTotal);
    makeSegmentImage(segmentImage);
    makeDisparityImage(disparityImage);
}

void StereoSlic::segment(const int superpixelTotal,
                         const png::image<png::rgb_pixel>& leftImage,
                         const png::image<png::gray_pixel_16>& leftDisparityImage,
                         const png::image<png::gray_pixel_16>& rightDisparityImage,
                         const png::image<png::rgb_pixel>& rightImage,
                         png::image<png::gray_pixel_16>& segmentImage,
                         png::image<png::gray_pixel_16>& disparityImage)
{
    energyType_ = 3;   // Color(left/right) + Disparity(left/right)
    setColorImage(leftImage, rightImage);
    setDisparityImage(leftDisparityImage, rightDisparityImage);
    performSegmentation(superpixelTotal);
    makeSegmentImage(segmentImage);
    makeDisparityImage(disparityImage);
}


void StereoSlic::setColorImage(const png::image<png::rgb_pixel>& leftImage) {
    width_ = static_cast<int>(leftImage.get_width());
    height_ = static_cast<int>(leftImage.get_height());
    
    convertRGBToLab(leftImage, leftLabImage_);
    calcLeftLabEdges();
}

void StereoSlic::setColorImage(const png::image<png::rgb_pixel>& leftImage,
                               const png::image<png::rgb_pixel>& rightImage)
{
    width_ = static_cast<int>(leftImage.get_width());
    height_ = static_cast<int>(leftImage.get_height());
    
    convertRGBToLab(leftImage, leftLabImage_);
    calcLeftLabEdges();
    convertRGBToLab(rightImage, rightLabImage_);
}

void StereoSlic::setDisparityImage(const png::image<png::gray_pixel_16>& leftDisparityImage) {
    leftDisparityImage_.resize(width_*height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            unsigned short pixelValue = leftDisparityImage[y][x];
            if (pixelValue == 0) leftDisparityImage_[width_*y + x] = -1.0;
            else leftDisparityImage_[width_*y + x] = static_cast<float>(pixelValue)/256.0f;
        }
    }
}

void StereoSlic::setDisparityImage(const png::image<png::gray_pixel_16>& leftDisparityImage,
                                   const png::image<png::gray_pixel_16>& rightDisparityImage)
{
    leftDisparityImage_.resize(width_*height_);
    rightDisparityImage_.resize(width_*height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            unsigned short leftPixelValue = leftDisparityImage[y][x];
            if (leftPixelValue == 0) leftDisparityImage_[width_*y + x] = -1.0;
            else leftDisparityImage_[width_*y + x] = static_cast<float>(leftPixelValue)/256.0f;

            unsigned short rightPixelValue = rightDisparityImage[y][x];
            if (rightPixelValue == 0) rightDisparityImage_[width_*y + x] = -1.0;
            else rightDisparityImage_[width_*y + x] = static_cast<float>(rightPixelValue)/256.0f;
        }
    }
}

void StereoSlic::performSegmentation(const int superpixelTotal) {
    initializeSeeds(superpixelTotal);
    
    for (int iterationCount = 0; iterationCount < iterationTotal_; ++iterationCount) {
        assignLabel();
        updateSeeds();
    }
    enforceLabelConnectivity();
}

void StereoSlic::makeSegmentImage(png::image<png::gray_pixel_16>& segmentImage) const {
    segmentImage.resize(width_, height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            segmentImage[y][x] = labels_[width_*y + x];
        }
    }
}

void StereoSlic::makeDisparityImage(png::image<png::gray_pixel_16>& disparityImage) {
    estimateDisparityPlaneParameter();

    disparityImage.resize(width_, height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int seedIndex = labels_[width_*y + x];
            double estimatedDisparity = seeds_[seedIndex].disparityPlane[0]*x + seeds_[seedIndex].disparityPlane[1]*y + seeds_[seedIndex].disparityPlane[2];
            if (estimatedDisparity > 0 && estimatedDisparity < 256.0) disparityImage[y][x] = static_cast<unsigned short>(estimatedDisparity*256.0 + 0.5);
            else if (estimatedDisparity >= 256.0) disparityImage[y][x] = 65535;
            else disparityImage[y][x] = 0;
        }
    }
}

void StereoSlic::convertRGBToLab(const png::image<png::rgb_pixel>& rgbImage,
                                 std::vector<float>& labImage)
{
    const int RGB2LABCONVERTER_XYZ_TABLE_SIZE = 1024;
    // CIE standard parameters
    const double epsilon = 0.008856;
    const double kappa = 903.3;
    // Reference white
    const double referenceWhite[3] = {0.950456, 1.0, 1.088754};
    /// Maximum values
    const double maxXYZValues[3] = {0.95047, 1.0, 1.08883};

    std::vector<float> sRGBGammaCorrections(256);
    for (int pixelValue = 0; pixelValue < 256; ++pixelValue) {
        double normalizedValue = pixelValue/255.0;
        double transformedValue = (normalizedValue <= 0.04045) ? normalizedValue/12.92 : pow((normalizedValue+0.055)/1.055, 2.4);
        
        sRGBGammaCorrections[pixelValue] = transformedValue;
    }
    
    int tableSize = RGB2LABCONVERTER_XYZ_TABLE_SIZE;
    std::vector<double> xyzTableIndexCoefficients(3);
    xyzTableIndexCoefficients[0] = (tableSize-1)/maxXYZValues[0];
    xyzTableIndexCoefficients[1] = (tableSize-1)/maxXYZValues[1];
    xyzTableIndexCoefficients[2] = (tableSize-1)/maxXYZValues[2];
    
    std::vector< std::vector<float> > fXYZConversions(3);
    for (int xyzIndex = 0; xyzIndex < 3; ++xyzIndex) {
        fXYZConversions[xyzIndex].resize(tableSize);
        double stepValue = maxXYZValues[xyzIndex]/tableSize;
        for (int tableIndex = 0; tableIndex < tableSize; ++tableIndex) {
            double originalValue = stepValue*tableIndex;
            double normalizedValue = originalValue/referenceWhite[xyzIndex];
            double transformedValue = (normalizedValue > epsilon) ? pow(normalizedValue, 1.0/3.0) : (kappa*normalizedValue + 16.0)/116.0;
            
            fXYZConversions[xyzIndex][tableIndex] = transformedValue;
        }
    }
    
    labImage.resize(width_*height_*3);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            png::rgb_pixel rgbColor = rgbImage[y][x];
            
            float correctedR = sRGBGammaCorrections[rgbColor.red];
            float correctedG = sRGBGammaCorrections[rgbColor.green];
            float correctedB = sRGBGammaCorrections[rgbColor.blue];
            
            float xyzColor[3];
            xyzColor[0] = correctedR*0.4124564f + correctedG*0.3575761f + correctedB*0.1804375f;
            xyzColor[1] = correctedR*0.2126729f + correctedG*0.7151522f + correctedB*0.0721750f;
            xyzColor[2] = correctedR*0.0193339f + correctedG*0.1191920f + correctedB*0.9503041f;
            
            int tableIndexX = static_cast<int>(xyzColor[0]*xyzTableIndexCoefficients[0] + 0.5);
            int tableIndexY = static_cast<int>(xyzColor[1]*xyzTableIndexCoefficients[1] + 0.5);
            int tableIndexZ = static_cast<int>(xyzColor[2]*xyzTableIndexCoefficients[2] + 0.5);
            
            float fX = fXYZConversions[0][tableIndexX];
            float fY = fXYZConversions[1][tableIndexY];
            float fZ = fXYZConversions[2][tableIndexZ];
            
            labImage[width_*3*y + 3*x] = 116.0*fY - 16.0;
            labImage[width_*3*y + 3*x + 1] = 500.0*(fX - fY);
            labImage[width_*3*y + 3*x + 2] = 200.0*(fY - fZ);
        }
    }
}

void StereoSlic::calcLeftLabEdges() {
    leftLabEdges_.resize(width_*height_, 0);
    for (int y = 1; y < height_-1; ++y) {
        for (int x = 1; x < width_-1; ++x) {
            double dxL = leftLabImage_[width_*3*y + 3*(x-1)] - leftLabImage_[width_*3*y + 3*(x+1)];
            double dxA = leftLabImage_[width_*3*y + 3*(x-1) + 1] - leftLabImage_[width_*3*y + 3*(x+1) + 1];
            double dxB = leftLabImage_[width_*3*y + 3*(x-1) + 2] - leftLabImage_[width_*3*y + 3*(x+1) + 2];
            double dx = dxL*dxL + dxA*dxA + dxB*dxB;
            
            double dyL = leftLabImage_[width_*3*(y-1) + 3*x] - leftLabImage_[width_*3*(y+1) + 3*x];
            double dyA = leftLabImage_[width_*3*(y-1) + 3*x + 1] - leftLabImage_[width_*3*(y+1) + 3*x + 1];
            double dyB = leftLabImage_[width_*3*(y-1) + 3*x + 2] - leftLabImage_[width_*3*(y+1) + 3*x + 2];
            double dy = dyL*dyL + dyA*dyA + dyB*dyB;
            
            leftLabEdges_[width_*y + x] = dx + dy;
        }
    }
}

void StereoSlic::initializeSeeds(const int superpixelTotal) {
    int imageSize = width_*height_;
    gridSize_ = sqrt(static_cast<double>(imageSize)/superpixelTotal);
    stepSize_ = static_cast<int>(gridSize_ + 2.0);
    int offsetX = static_cast<int>(gridSize_/2.0);
    int offsetY = static_cast<int>(gridSize_/2.0);
    
    seeds_.clear();
    for (int indexY = 0; indexY < height_; ++indexY) {
        int y = static_cast<int>(gridSize_*indexY + offsetY + 0.5);
        if (y >= height_) break;
        for (int indexX = 0; indexX < width_; ++indexX) {
            int x = static_cast<int>(gridSize_*indexX + offsetX + 0.5);
            if (x >= width_) break;
            
            LabXYD newSeed;
            newSeed.color[0] = leftLabImage_[width_*3*y + 3*x];
            newSeed.color[1] = leftLabImage_[width_*3*y + 3*x + 1];
            newSeed.color[2] = leftLabImage_[width_*3*y + 3*x + 2];
            newSeed.position[0] = x;
            newSeed.position[1] = y;
            seeds_.push_back(newSeed);
        }
    }
    
    int seedTotal = static_cast<int>(seeds_.size());
    for (int seedIndex = 0; seedIndex < seedTotal; ++seedIndex) {
        int originalX = seeds_[seedIndex].position[0];
        int originalY = seeds_[seedIndex].position[1];
        int originalPixelIndex = width_*originalY + originalX;
        
        int perturbedPixelIndex = originalPixelIndex;
        for (int neighborIndex = 0; neighborIndex < eightNeighborTotal; ++neighborIndex) {
            int neighborX = originalX + eightNeighborOffsetX[neighborIndex];
            int neighborY = originalY + eightNeighborOffsetY[neighborIndex];
            if (neighborX < 0 || neighborX >= width_ || neighborY < 0 || neighborY >= height_) continue;
            int neighborPixelIndex = width_*neighborY + neighborX;
            
            if (leftLabEdges_[neighborPixelIndex] < leftLabEdges_[perturbedPixelIndex]) {
                perturbedPixelIndex = neighborPixelIndex;
            }
        }
        
        if (perturbedPixelIndex != originalPixelIndex) {
            int perturbedX = perturbedPixelIndex%width_;
            int perturbedY = perturbedPixelIndex/width_;
            
            seeds_[seedIndex].color[0] = leftLabImage_[width_*3*perturbedY + 3*perturbedX];
            seeds_[seedIndex].color[1] = leftLabImage_[width_*3*perturbedY + 3*perturbedX + 1];
            seeds_[seedIndex].color[2] = leftLabImage_[width_*3*perturbedY + 3*perturbedX + 2];
            seeds_[seedIndex].position[0] = perturbedX;
            seeds_[seedIndex].position[1] = perturbedY;
        }
    }
}

void StereoSlic::assignLabel() {
    int seedTotal = static_cast<int>(seeds_.size());
    
    double positionWeight = colorWeight_/(stepSize_*stepSize_);
    
    labels_.resize(width_*height_, -1);
    std::vector<double> distancesToSeeds(width_*height_, DBL_MAX);
    for (int seedIndex = 0; seedIndex < seedTotal; ++seedIndex) {
        int minX = (seeds_[seedIndex].position[0] > stepSize_) ? seeds_[seedIndex].position[0] - stepSize_ : 0;
        int maxX = (seeds_[seedIndex].position[0] < width_ - stepSize_) ? seeds_[seedIndex].position[0] + stepSize_ : width_;
        int minY = (seeds_[seedIndex].position[1] > stepSize_) ? seeds_[seedIndex].position[1] - stepSize_ : 0;
        int maxY = (seeds_[seedIndex].position[1] < height_ - stepSize_) ? seeds_[seedIndex].position[1] + stepSize_ : height_;
        
        double seedL = seeds_[seedIndex].color[0];
        double seedA = seeds_[seedIndex].color[1];
        double seedB = seeds_[seedIndex].color[2];
        double seedX = seeds_[seedIndex].position[0];
        double seedY = seeds_[seedIndex].position[1];
        double seedAlpha = seeds_[seedIndex].disparityPlane[0];
        double seedBeta = seeds_[seedIndex].disparityPlane[1];
        double seedGamma = seeds_[seedIndex].disparityPlane[2];
        
        for (int y = minY; y < maxY; ++y) {
            for (int x = minX; x < maxX; ++x) {
                float leftPixelL = leftLabImage_[width_*3*y + 3*x];
                float leftPixelA = leftLabImage_[width_*3*y + 3*x + 1];
                float leftPixelB = leftLabImage_[width_*3*y + 3*x + 2];
                double distanceLeftLab = (leftPixelL - seedL)*(leftPixelL - seedL)
                    + (leftPixelA - seedA)*(leftPixelA - seedA)
                    + (leftPixelB - seedB)*(leftPixelB - seedB);
                double distanceXY = (x - seedX)*(x - seedX) + (y - seedY)*(y - seedY);
                
                // Default distances
                double distanceRightLab = distanceLeftLab;
                double distanceLeftD = noDisparityPenalty_;
                double distanceRightD = noDisparityPenalty_;
                
                if (energyType_ > 0) {
                    double estimatedDisparity = seedAlpha*x + seedBeta*y + seedGamma;
                    if (estimatedDisparity > 0) {
                        if (leftDisparityImage_[width_*y + x] > 0) {
                            // distance of left disparities
                            double leftDisparityDifference = leftDisparityImage_[width_*y + x] - estimatedDisparity;
                            distanceLeftD = leftDisparityDifference*leftDisparityDifference;
                        }
                        
                        if (energyType_ > 1) {
                            int rightX = static_cast<int>(x - estimatedDisparity + 0.5);
                            if (rightX >= 0 && rightDisparityImage_[width_*y + rightX] > 0) {
                                double rightDisparityDifference = estimatedDisparity - rightDisparityImage_[width_*y + rightX];
                                if (rightDisparityDifference < -1) {
                                    // Occluded
                                    rightDisparityDifference = 0;
                                    distanceRightLab = distanceLeftLab;
                                } else {
                                    // Non-occluded
                                    distanceRightD = rightDisparityDifference*rightDisparityDifference;
                                    if (energyType_ > 2) {
                                        float rightPixelL = rightLabImage_[width_*3*y + 3*rightX];
                                        float rightPixelA = rightLabImage_[width_*3*y + 3*rightX + 1];
                                        float rightPixelB = rightLabImage_[width_*3*y + 3*rightX + 2];
                                        distanceRightLab = (rightPixelL - seedL)*(rightPixelL - seedL)
                                            + (rightPixelA - seedA)*(rightPixelA - seedA)
                                            + (rightPixelB - seedB)*(rightPixelB - seedB);
                                    }
                                }
                            }
                        }
                    }
                }
                
                double distanceLab = distanceLeftLab;
                if (energyType_ == 3) distanceLab += distanceRightLab;
                double distanceD = 0;
                if (energyType_ > 0) distanceD = distanceLeftD;
                if (energyType_ > 1) distanceD += distanceRightD;
                
                double distanceLabXYD = distanceLab + positionWeight*distanceXY + disparityWeight_*distanceD;
                if (distanceLabXYD < distancesToSeeds[width_*y + x]) {
                    distancesToSeeds[width_*y + x] = distanceLabXYD;
                    labels_[width_*y + x] = seedIndex;
                }
            }
        }
    }
}

void StereoSlic::updateSeeds() {
    updateSeedsColorAndPosition();
    if (energyType_ > 0) {
        estimateDisparityPlaneParameter();
    }
}

void StereoSlic::updateSeedsColorAndPosition() {
    int seedTotal = static_cast<int>(seeds_.size());
    
    std::vector<int> segmentSizes(seedTotal, 0);
    std::vector<LabXYD> segmentSigmas(seedTotal);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int pixelSeedIndex = labels_[width_*y + x];
            if (pixelSeedIndex >= 0) {
                segmentSigmas[pixelSeedIndex].color[0] += leftLabImage_[width_*3*y + 3*x];
                segmentSigmas[pixelSeedIndex].color[1] += leftLabImage_[width_*3*y + 3*x + 1];
                segmentSigmas[pixelSeedIndex].color[2] += leftLabImage_[width_*3*y + 3*x + 2];
                segmentSigmas[pixelSeedIndex].position[0] += x;
                segmentSigmas[pixelSeedIndex].position[1] += y;
                ++segmentSizes[pixelSeedIndex];
            }
        }
    }
    
    for (int seedIndex = 0; seedIndex < seedTotal; ++seedIndex) {
        double segmentSizeInverse = 1.0;
        if (segmentSizes[seedIndex] > 0) segmentSizeInverse = 1.0/segmentSizes[seedIndex];
        
        seeds_[seedIndex].color[0] = segmentSigmas[seedIndex].color[0]*segmentSizeInverse;
        seeds_[seedIndex].color[1] = segmentSigmas[seedIndex].color[1]*segmentSizeInverse;
        seeds_[seedIndex].color[2] = segmentSigmas[seedIndex].color[2]*segmentSizeInverse;
        seeds_[seedIndex].position[0] = segmentSigmas[seedIndex].position[0]*segmentSizeInverse;
        seeds_[seedIndex].position[1] = segmentSigmas[seedIndex].position[1]*segmentSizeInverse;
    }
}

void StereoSlic::estimateDisparityPlaneParameter() {
    std::vector< std::vector<DisparityPixel> > segmentDisparityPixels = makeDisparityPixelList();
    int seedTotal = static_cast<int>(segmentDisparityPixels.size());
    
    for (int seedIndex = 0; seedIndex < seedTotal; ++seedIndex) {
        std::vector<double> planeParameter = estimateDisparityPlaneParameter(segmentDisparityPixels[seedIndex]);
        
        seeds_[seedIndex].disparityPlane[0] = planeParameter[0];
        seeds_[seedIndex].disparityPlane[1] = planeParameter[1];
        seeds_[seedIndex].disparityPlane[2] = planeParameter[2];
    }
}

std::vector< std::vector<StereoSlic::DisparityPixel> > StereoSlic::makeDisparityPixelList() const {
    int seedTotal = static_cast<int>(seeds_.size());
    
    std::vector< std::vector<DisparityPixel> > segmentDisparityPixels(seedTotal);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (leftDisparityImage_[width_*y + x] > 0) {
                int pixelSeedIndex = labels_[width_*y + x];
                if (pixelSeedIndex >= 0) {
                    DisparityPixel newDisparityPixel;
                    newDisparityPixel.x = x;
                    newDisparityPixel.y = y;
                    newDisparityPixel.d = leftDisparityImage_[width_*y + x];
                    segmentDisparityPixels[pixelSeedIndex].push_back(newDisparityPixel);
                }
            }
        }
    }
    
    return segmentDisparityPixels;
}

std::vector<double> StereoSlic::estimateDisparityPlaneParameter(const std::vector<DisparityPixel>& disparityPixels) const {
    std::vector<double> planeParameter(3);
    
    int pixelTotal = static_cast<int>(disparityPixels.size());
    if (pixelTotal < 3) {
        planeParameter[0] = 0;
        planeParameter[1] = 0;
        planeParameter[2] = -1;
    } else {
        bool isSameX = true;
        bool isSameY = true;
        for (int pixelIndex = 1; pixelIndex < pixelTotal; ++pixelIndex) {
            if (disparityPixels[pixelIndex].x != disparityPixels[0].x) isSameX = false;
            if (disparityPixels[pixelIndex].y != disparityPixels[0].y) isSameY = false;
            if (!isSameX && !isSameY) break;
        }
        if (isSameX || isSameY) {
            double disparitySum = 0.0;
            for (int pixelIndex = 0; pixelIndex < pixelTotal; ++pixelIndex) disparitySum += disparityPixels[pixelIndex].d;
            
            planeParameter[0] = 0;
            planeParameter[1] = 0;
            planeParameter[2] = -1;
        } else {
            planeParameter = estimateDisparityPlaneParameterRANSAC(disparityPixels);
        }
    }
    
    return planeParameter;
}

std::vector<double> StereoSlic::estimateDisparityPlaneParameterRANSAC(const std::vector<DisparityPixel>& disparityPixels) const {
    const double inlierThreshold = 2.0;
    const double confidenceLevel = 0.99;
    
    int pixelTotal = static_cast<int>(disparityPixels.size());
    
    int samplingTotal = pixelTotal*2;
    
    int bestInlierTotal = 0;
    std::vector<bool> bestInlierFlags(pixelTotal);
    int samplingCount = 0;
    while (samplingCount < samplingTotal) {
        // Randomly select 3 pixels
        int drawIndices[3];
        drawIndices[0] = rand()%pixelTotal;
        drawIndices[1] = rand()%pixelTotal;
        while(drawIndices[1] == drawIndices[0]) drawIndices[1] = rand()%pixelTotal;
        drawIndices[2] = rand()%pixelTotal;
        while(drawIndices[2] == drawIndices[1] || drawIndices[2] == drawIndices[0]
              || (disparityPixels[drawIndices[0]].x == disparityPixels[drawIndices[1]].x && disparityPixels[drawIndices[0]].x == disparityPixels[drawIndices[2]].x)
              || (disparityPixels[drawIndices[0]].y == disparityPixels[drawIndices[1]].y && disparityPixels[drawIndices[0]].y == disparityPixels[drawIndices[2]].y))
        {
            drawIndices[2] = rand()%pixelTotal;
        }
        
        // Compute plane parameters
        Eigen::Matrix3d matPosition;
        Eigen::Vector3d vecDisparity;
        for (int i = 0; i < 3; ++i) {
            matPosition(i, 0) = disparityPixels[drawIndices[i]].x;
            matPosition(i, 1) = disparityPixels[drawIndices[i]].y;
            matPosition(i, 2) = 1.0;
            vecDisparity(i) = disparityPixels[drawIndices[i]].d;
        }
        Eigen::Vector3d planeParameter = matPosition.colPivHouseholderQr().solve(vecDisparity);
        
        // Count the number of inliers
        int inlierTotal = 0;
        std::vector<bool> inlierFlags(pixelTotal);
        for (int pixelIndex = 0; pixelIndex < pixelTotal; ++pixelIndex) {
            double estimatedDisparity = planeParameter(0)*disparityPixels[pixelIndex].x + planeParameter(1)*disparityPixels[pixelIndex].y + planeParameter(2);
            if (fabs(estimatedDisparity - disparityPixels[pixelIndex].d) <= inlierThreshold) {
                ++inlierTotal;
                inlierFlags[pixelIndex] = true;
            } else {
                inlierFlags[pixelIndex] = false;
            }
        }
        
        // Update best inliers
        if (inlierTotal > bestInlierTotal) {
            bestInlierTotal = inlierTotal;
            bestInlierFlags = inlierFlags;
            
            samplingTotal = computeRequiredSamplingTotal(3, bestInlierTotal, pixelTotal, samplingTotal, confidenceLevel);
        }
        
        // Increment
        ++samplingCount;
    }
    
    // Least-square estimation
    Eigen::MatrixXd matPosition(bestInlierTotal, 3);
    Eigen::VectorXd vecDisparity(bestInlierTotal);
    int inlierIndex = 0;
    for (int pixelIndex = 0; pixelIndex < pixelTotal; ++pixelIndex) {
        if (bestInlierFlags[pixelIndex]) {
            matPosition(inlierIndex, 0) = disparityPixels[pixelIndex].x;
            matPosition(inlierIndex, 1) = disparityPixels[pixelIndex].y;
            matPosition(inlierIndex, 2) = 1.0;
            vecDisparity(inlierIndex) = disparityPixels[pixelIndex].d;
            ++inlierIndex;
        }
    }
    Eigen::Vector3d vecPlaneParameter = matPosition.colPivHouseholderQr().solve(vecDisparity);
    std::vector<double> planeParameter(3);
    planeParameter[0] = vecPlaneParameter(0);
    planeParameter[1] = vecPlaneParameter(1);
    planeParameter[2] = vecPlaneParameter(2);
    
    return planeParameter;
}

void StereoSlic::enforceLabelConnectivity() {
    int imageSize = width_*height_;
    int seedTotal = static_cast<int>(seeds_.size());
    
    const int minimumSegmentSizeThreshold = imageSize/seedTotal/4;
    
    std::vector<int> newLabels(width_*height_, -1);
    int newLabelIndex = 0;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (newLabels[width_*y + x] >= 0) continue;
            
            newLabels[width_*y + x] = newLabelIndex;
            
            int adjacentLabel = 0;
            for (int neighborIndex = 0; neighborIndex < fourNeighborTotal; ++neighborIndex) {
                int neighborX = x + fourNeighborOffsetX[neighborIndex];
                int neighborY = y + fourNeighborOffsetY[neighborIndex];
                if (neighborX < 0 || neighborX >= width_ || neighborY < 0 || neighborY >= height_) continue;
                
                if (newLabels[width_*neighborY + neighborX] >= 0) adjacentLabel = newLabels[width_*neighborY + neighborX];
            }
            
            std::vector<int> connectedXs(1);
            std::vector<int> connectedYs(1);
            connectedXs[0] = x;
            connectedYs[0] = y;
            labelConnectedPixels(x, y, newLabelIndex, newLabels, connectedXs, connectedYs);
            
            int segmentPixelTotal = static_cast<int>(connectedXs.size());
            if (segmentPixelTotal <= minimumSegmentSizeThreshold) {
                for (int i = 0; i < segmentPixelTotal; ++i) {
                    newLabels[width_*connectedYs[i] + connectedXs[i]] = adjacentLabel;
                }
                --newLabelIndex;
            }
            
            ++newLabelIndex;
        }
    }
    
    labels_ = newLabels;
    seeds_.resize(newLabelIndex);
}

void StereoSlic::labelConnectedPixels(const int x, const int y, const int newLabelIndex,
                                      std::vector<int>& newLabels, std::vector<int>& connectedXs, std::vector<int>& connectedYs) const
{
    int originalLabelIndex = labels_[width_*y + x];
    for (int neighborIndex = 0; neighborIndex < fourNeighborTotal; ++neighborIndex) {
        int neighborX = x + fourNeighborOffsetX[neighborIndex];
        int neighborY = y + fourNeighborOffsetY[neighborIndex];
        if (neighborX < 0 || neighborX >= width_ || neighborY < 0 || neighborY >= height_) continue;
        
        if (newLabels[width_*neighborY + neighborX] < 0 && labels_[width_*neighborY + neighborX] == originalLabelIndex) {
            connectedXs.push_back(neighborX);
            connectedYs.push_back(neighborY);
            newLabels[width_*neighborY + neighborX] = newLabelIndex;
            labelConnectedPixels(neighborX, neighborY, newLabelIndex, newLabels, connectedXs, connectedYs);
        }
    }
}


int computeRequiredSamplingTotal(const int drawTotal, const int inlierTotal, const int pointTotal, const int currentSamplingTotal, const double confidenceLevel) {
    double ep = 1 - static_cast<double>(inlierTotal)/static_cast<double>(pointTotal);
    if (ep == 1.0) {
        ep = 0.5;
    }
    
    int newSamplingTotal = static_cast<int>(log(1 - confidenceLevel)/log(1 - pow(1 - ep, drawTotal)) + 0.5);
    if (newSamplingTotal < currentSamplingTotal) {
        return newSamplingTotal;
    } else {
        return currentSamplingTotal;
    }
}


png::image<png::rgb_pixel> drawSegmentBoundary(const png::image<png::rgb_pixel>& originalImage,
                                               const png::image<png::gray_pixel_16>& segmentImage,
                                               const png::rgb_pixel boundaryColor)
{
    // Pixel offsets of 8-neighbors
    const int eightNeighborTotal = 8;
    const int eightNeighborOffsetX[8] = {-1,-1, 0, 1, 1, 1, 0,-1};
    const int eightNeighborOffsetY[8] = { 0,-1,-1,-1, 0, 1, 1, 1};
    
    // Check image size
    int width = static_cast<int>(originalImage.get_width());
    int height = static_cast<int>(originalImage.get_height());
    if (segmentImage.get_width() != width || segmentImage.get_height() != height) {
        throw std::runtime_error("drawSegmentBoundary: image sizes are different");
    }
    
    // Copy input image
    png::image<png::rgb_pixel> boundaryImage(originalImage);
    
    // Draw boundary
    std::vector<bool> boundaryFlags(width*height, false);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int pixelLabelIndex = segmentImage[y][x];
            
            bool drawBoundary = false;
            for (int neighborIndex = 0; neighborIndex < eightNeighborTotal; ++neighborIndex) {
                int neighborX = x + eightNeighborOffsetX[neighborIndex];
                int neighborY = y + eightNeighborOffsetY[neighborIndex];
                if (neighborX < 0 || neighborX >= width || neighborY < 0 || neighborY >= height) continue;
                
                int neighborPixelIndex = width*neighborY + neighborX;
                if (boundaryFlags[neighborPixelIndex]) continue;
                if (segmentImage[neighborY][neighborX] != pixelLabelIndex) {
                    drawBoundary = true;
                    break;
                }
            }
            
            if (drawBoundary) {
                boundaryImage[y][x] = boundaryColor;
            }
        }
    }
    
    return boundaryImage;
}

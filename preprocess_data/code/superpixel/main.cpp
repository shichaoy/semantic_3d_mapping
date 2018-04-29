#include <iostream>
#include <png++/png.hpp>
#include "cmdline.h"
#include "StereoSlic.h"
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <fstream>

struct ParameterStereoSlic {
    bool verbose;
    int energyType;
    int superpixelTotal;
    int iterationTotal;
    double colorWeight;
    double disparityWeight;
    double noDisparityPenalty;
    std::string leftImageFilename;
    std::string rightImageFilename;
    std::string leftDisparityImageFilename;
    std::string rightDisparityImageFilename;
    std::string outputSegmentImageFilename;
    std::string outputDisparityImageFilename;
    std::string outputBoundaryImageFilename;
};

// Prototype declaration
cmdline::parser makeCommandParser();
ParameterStereoSlic parseCommandline(int argc, char* argv[]);

cmdline::parser makeCommandParser() {
    cmdline::parser commandParser;
    commandParser.add<std::string>("output", 'o', "output segment file", false, "");
    commandParser.add<int>("superpixel", 's', "the number of superpixels", false, 1000);
    commandParser.add<int>("iteration", 'i', "the number of iterations", false, 10);
    commandParser.add<double>("color", 'c', "weight of color term", false, 3000);
    commandParser.add<double>("disparity", 'd', "weight of disparity term", false, 30);
    commandParser.add<double>("penalty", 'p', "penalty value of disparity term without disparity estimation", false, 3.0);
    commandParser.add("verbose", 'v', "verbose");
    commandParser.add("help", 'h', "display this message");
    commandParser.footer("left_image [left_disparity_image [right_disparity_image [right_image]]]");
    commandParser.set_program_name("stereoslic");
    
    return commandParser;
}

ParameterStereoSlic parseCommandline(int argc, char* argv[]) {
    // Make command parser
    cmdline::parser commandParser = makeCommandParser();
    // Parse command line
    bool isCorrectCommandline = commandParser.parse(argc, argv);
    // Check arguments
    if (!isCorrectCommandline) {
        std::cerr << commandParser.error() << std::endl;
    }
    if (!isCorrectCommandline || commandParser.exist("help") || commandParser.rest().size() < 1) {
        std::cerr << commandParser.usage() << std::endl;
        exit(1);
    }
    
    // Set program parameters
    ParameterStereoSlic parameters;
    // Verbose flag
    parameters.verbose = commandParser.exist("verbose");
    // Type of energy function
    if (commandParser.rest().size() > 3) parameters.energyType = 3;
    else if (commandParser.rest().size() > 2) parameters.energyType = 2;
    else if (commandParser.rest().size() > 1) parameters.energyType = 1;
    else parameters.energyType = 0;
    // The number of superpixels
    parameters.superpixelTotal = commandParser.get<int>("superpixel");
    // The number of iterations
    parameters.iterationTotal = commandParser.get<int>("iteration");
    // Color weight
    parameters.colorWeight = commandParser.get<double>("color");
    // Disparity weight
    parameters.disparityWeight = commandParser.get<double>("disparity");
    // Occluded penalty
    parameters.noDisparityPenalty = commandParser.get<double>("penalty");
    // Input files
    parameters.leftImageFilename = commandParser.rest()[0];
    if (parameters.energyType > 0) parameters.leftDisparityImageFilename = commandParser.rest()[1];
    else parameters.leftDisparityImageFilename = "";
    if (parameters.energyType > 1) parameters.rightDisparityImageFilename = commandParser.rest()[2];
    else parameters.rightDisparityImageFilename = "";
    if (parameters.energyType > 2) parameters.rightImageFilename = commandParser.rest()[3];
    else parameters.rightImageFilename = "";
    // Output files
    std::string outputSegmentImageFilename = commandParser.get<std::string>("output");
    if (outputSegmentImageFilename == "") {
        outputSegmentImageFilename = parameters.leftImageFilename;
        size_t dotPosition = outputSegmentImageFilename.rfind('.');
        if (dotPosition != std::string::npos) outputSegmentImageFilename.erase(dotPosition);
        outputSegmentImageFilename += "_slic.png";
    }
    parameters.outputSegmentImageFilename = outputSegmentImageFilename;
    std::string outputDisparityImageFilename = outputSegmentImageFilename;
    size_t dotPosition = outputDisparityImageFilename.rfind('.');
    if (dotPosition != std::string::npos) outputDisparityImageFilename.erase(dotPosition);
    outputDisparityImageFilename += "_disparity.png";
    parameters.outputDisparityImageFilename = outputDisparityImageFilename;
    std::string outputBoundaryImageFilename = outputSegmentImageFilename;
    dotPosition = outputBoundaryImageFilename.rfind('.');
    if (dotPosition != std::string::npos) outputBoundaryImageFilename.erase(dotPosition);
    outputBoundaryImageFilename += "_boundary.png";
    parameters.outputBoundaryImageFilename = outputBoundaryImageFilename;
    
    return parameters;
}


int batch_process_kitti(int argc, char* argv[]){
    ParameterStereoSlic parameters = parseCommandline(argc, argv);

    std::string image_folder = "../../raw_imgs/";
    std::string output_folder = "../../";
    std::string image_list_file = image_folder+"namelist_small.txt";

    std::ifstream infile(image_list_file.c_str());
    
    std::vector<std::string> filenames;
    for( std::string line; std::getline( infile, line ); )
    {
        std::stringstream trimmer;
        trimmer << line;
        line.clear();
        trimmer >> line;
        
        if (line.size() > 0)
            filenames.push_back( line );
    }
    std::cout<<"Total images "<<filenames.size()<<std::endl;
    std::cout<<"superpixelTotal "<<parameters.superpixelTotal<<std::endl;
    
    for (uint i=0; i<filenames.size(); i++) {
        // std::cout<<"processing "<<i<<std::endl;
        std::string image_name = image_folder +"image_2/" + filenames[i];
        std::string output_image_file_name = output_folder + "superpixel_ind/"+filenames[i];
        std::string output_boundary_image_filename = output_folder+"superpixel_img/"+ filenames[i];
        std::cout<<"Processing "<<filenames[i]<<std::endl;
        
        try {           
            StereoSlic stereoSlic;
            stereoSlic.setIterationTotal(parameters.iterationTotal);
            stereoSlic.setEnergyParameter(parameters.colorWeight, parameters.disparityWeight, parameters.noDisparityPenalty);

            png::image<png::gray_pixel_16> segmentImage;
            png::image<png::gray_pixel_16> disparityImage;
            
            png::image<png::rgb_pixel> leftImage(image_name); // seems that 
            
            clock_t begin1 = clock();
            stereoSlic.segment(parameters.superpixelTotal, leftImage, segmentImage);
            std::cout<<"slic take time:  "<< double(clock() - begin1) / CLOCKS_PER_SEC<<std::endl;
            // std::cout<<double(clock() - begin1) / CLOCKS_PER_SEC<<std::endl;

            png::image<png::rgb_pixel> boundaryImage = drawSegmentBoundary(leftImage, segmentImage);
            segmentImage.write(output_image_file_name);
            boundaryImage.write(output_boundary_image_filename);        
        } catch (const std::exception& exception) {
            std::cerr << "Error: " << exception.what() << std::endl;
            exit(1);
        }
    }
    return 1;
}


int main(int argc, char* argv[]) {
  
    return batch_process_kitti(argc, argv);
    
    // Parse command line
    ParameterStereoSlic parameters = parseCommandline(argc, argv);

    if (parameters.verbose) {
        std::cerr << std::endl;
        std::cerr << "Left image:             " << parameters.leftImageFilename << std::endl;
        if (parameters.energyType > 0) std::cerr << "Left disparity image:   " << parameters.leftDisparityImageFilename << std::endl;
        if (parameters.energyType > 1) std::cerr << "Right image:            " << parameters.rightImageFilename << std::endl;
        if (parameters.energyType > 2) std::cerr << "Right disparity image:  " << parameters.rightDisparityImageFilename << std::endl;
        std::cerr << "Output segment image:   " << parameters.outputSegmentImageFilename << std::endl;
        if (parameters.energyType > 0) std::cerr << "Output disparity image: " << parameters.outputDisparityImageFilename << std::endl;
        std::cerr << "Output boundary image:  " << parameters.outputBoundaryImageFilename << std::endl;
        std::cerr << "   Energy type:      ";
        if (parameters.energyType == 0) std::cerr << "color(left)" << std::endl;
        else if (parameters.energyType == 1) std::cerr << "color(left) + disparity(left)" << std::endl;
        else if (parameters.energyType == 2) std::cerr << "color(left) + disparity(left/right)" << std::endl;
        else std::cerr << "color(left/right) + disparity(left/right)" << std::endl;
        std::cerr << "   #superpixels:     " << parameters.superpixelTotal << std::endl;
        std::cerr << "   #iterations:      " << parameters.iterationTotal << std::endl;
        std::cerr << "   color weight:     " << parameters.colorWeight << std::endl;
        std::cerr << "   dispairty weight: " << parameters.disparityWeight << std::endl;
        std::cerr << "   no data penalty:  " << parameters.noDisparityPenalty << std::endl;
        std::cerr << std::endl;
    }
    
    try {
        clock_t begin1 = clock();
        StereoSlic stereoSlic;
        stereoSlic.setIterationTotal(parameters.iterationTotal);
        stereoSlic.setEnergyParameter(parameters.colorWeight, parameters.disparityWeight, parameters.noDisparityPenalty);
        
        png::image<png::gray_pixel_16> segmentImage;
        png::image<png::gray_pixel_16> disparityImage;
        
        png::image<png::rgb_pixel> leftImage(parameters.leftImageFilename);
        if (parameters.energyType == 0) {
            stereoSlic.segment(parameters.superpixelTotal, leftImage, segmentImage);
        } else if (parameters.energyType == 1) {
            png::image<png::gray_pixel_16> leftDisparityImage(parameters.leftDisparityImageFilename);
            stereoSlic.segment(parameters.superpixelTotal, leftImage, leftDisparityImage, segmentImage, disparityImage);
        } else if (parameters.energyType == 2) {
            png::image<png::gray_pixel_16> leftDisparityImage(parameters.leftDisparityImageFilename);
            png::image<png::gray_pixel_16> rightDisparityImage(parameters.rightDisparityImageFilename);
            stereoSlic.segment(parameters.superpixelTotal, leftImage, leftDisparityImage, rightDisparityImage, segmentImage, disparityImage);
        } else {
            png::image<png::gray_pixel_16> leftDisparityImage(parameters.leftDisparityImageFilename);
            png::image<png::gray_pixel_16> rightDisparityImage(parameters.rightDisparityImageFilename);
            png::image<png::rgb_pixel> rightImage(parameters.rightImageFilename);
            stereoSlic.segment(parameters.superpixelTotal,
                               leftImage, leftDisparityImage,
                               rightDisparityImage, rightImage,
                               segmentImage, disparityImage);
        }
        png::image<png::rgb_pixel> boundaryImage = drawSegmentBoundary(leftImage, segmentImage);
        
        segmentImage.write(parameters.outputSegmentImageFilename);
        if (parameters.energyType > 0) disparityImage.write(parameters.outputDisparityImageFilename);
        boundaryImage.write(parameters.outputBoundaryImageFilename);

        std::cout<<"slic take time:  "<< double(clock() - begin1) / CLOCKS_PER_SEC<<std::endl;

    } catch (const std::exception& exception) {
        std::cerr << "Error: " << exception.what() << std::endl;
        exit(1);
    }
}

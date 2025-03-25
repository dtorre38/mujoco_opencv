#include <opencv2/opencv.hpp>
#include <filesystem>

void create_disparity_map(cv::Mat stereo_img, cv::Mat& disparity_map, int frame_width, int frame_height){
    // Create data directory if it doesn't exist
    std::filesystem::create_directory("data");
    std::filesystem::create_directory("data/comparison");

    // Extract left and right from stereo image
    cv::Mat left_img = stereo_img(cv::Rect(0, 0, frame_width/2, frame_height));
    cv::Mat right_img = stereo_img(cv::Rect(frame_width/2, 0, frame_width/2, frame_height));

    // Save original stereo image
    cv::Mat stereo_flip;
    cv::flip(stereo_img, stereo_flip, 0);
    cv::imwrite("data/comparison/1_stereo_original.png", stereo_flip);

    // Save original left and right images
    cv::Mat left_img_flip, right_img_flip;
    cv::flip(left_img, left_img_flip, 0);
    cv::flip(right_img, right_img_flip, 0);
    cv::imwrite("data/comparison/2_left_original.png", left_img_flip);
    cv::imwrite("data/comparison/2_right_original.png", right_img_flip);

    // Convert to grayscale
    cv::Mat gray_left, gray_right;
    cv::cvtColor(left_img, gray_left, cv::COLOR_RGB2GRAY);
    cv::cvtColor(right_img, gray_right, cv::COLOR_RGB2GRAY);
    
    // Save grayscale images
    cv::Mat gray_left_flip, gray_right_flip;
    cv::flip(gray_left, gray_left_flip, 0);
    cv::flip(gray_right, gray_right_flip, 0);
    cv::imwrite("data/comparison/3_left_grayscale.png", gray_left_flip);
    cv::imwrite("data/comparison/3_right_grayscale.png", gray_right_flip);

    // Make copies for comparison between methods
    cv::Mat gray_left_orig = gray_left.clone();
    cv::Mat gray_right_orig = gray_right.clone();

    // Enhance image contrast with histogram equalization
    cv::equalizeHist(gray_left, gray_left);
    cv::equalizeHist(gray_right, gray_right);
    
    // Save histogram equalized images
    cv::Mat hist_left_flip, hist_right_flip;
    cv::flip(gray_left, hist_left_flip, 0);
    cv::flip(gray_right, hist_right_flip, 0);
    cv::imwrite("data/comparison/4_left_histeq.png", hist_left_flip);
    cv::imwrite("data/comparison/4_right_histeq.png", hist_right_flip);

    // Denoise using Gaussian blur
    cv::Mat blur_left = gray_left.clone();
    cv::Mat blur_right = gray_right.clone();
    cv::GaussianBlur(gray_left, blur_left, cv::Size(5, 5), 1.5);
    cv::GaussianBlur(gray_right, blur_right, cv::Size(5, 5), 1.5);
    
    // Save blurred images
    cv::Mat blur_left_flip, blur_right_flip;
    cv::flip(blur_left, blur_left_flip, 0);
    cv::flip(blur_right, blur_right_flip, 0);
    cv::imwrite("data/comparison/5_left_blur.png", blur_left_flip);
    cv::imwrite("data/comparison/5_right_blur.png", blur_right_flip);

    int numDisparities = 16;   // Must be divisible by 16
    int blockSize = 9;         // Typically odd numbers between 5-15 work best

    // APPROACH 1: Simple StereoBM (like in the Python example)
    cv::Ptr<cv::StereoBM> stereoBM_simple = cv::StereoBM::create(numDisparities, 15);
    cv::Mat disparity_simple;
    stereoBM_simple->compute(gray_left_orig, gray_right_orig, disparity_simple);
    
    // Save simple disparity result
    cv::Mat disparity_simple_normalized;
    cv::normalize(disparity_simple, disparity_simple_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat disparity_simple_flip;
    cv::flip(disparity_simple_normalized, disparity_simple_flip, 0);
    cv::imwrite("data/comparison/6_disparity_stereobm_simple.png", disparity_simple_flip);

    // APPROACH 2: StereoBM with fine tuning
    cv::Ptr<cv::StereoBM> stereoBM_tuned = cv::StereoBM::create(64, 9);
    stereoBM_tuned->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
    stereoBM_tuned->setPreFilterSize(9);
    stereoBM_tuned->setPreFilterCap(31);
    stereoBM_tuned->setMinDisparity(0);
    stereoBM_tuned->setTextureThreshold(10);
    stereoBM_tuned->setUniquenessRatio(15);
    stereoBM_tuned->setSpeckleRange(32);
    stereoBM_tuned->setSpeckleWindowSize(100);
    
    // Version 2a: StereoBM tuned on grayscale only
    cv::Mat disparity_bm_gray;
    stereoBM_tuned->compute(gray_left_orig, gray_right_orig, disparity_bm_gray);
    
    // Version 2b: StereoBM tuned on histeq images
    cv::Mat disparity_bm_histeq;
    stereoBM_tuned->compute(gray_left, gray_right, disparity_bm_histeq);
    
    // Version 2c: StereoBM tuned on blurred images
    cv::Mat disparity_bm_blur;
    stereoBM_tuned->compute(blur_left, blur_right, disparity_bm_blur);
    
    // Save all StereoBM results
    cv::Mat disparity_bm_gray_norm, disparity_bm_histeq_norm, disparity_bm_blur_norm;
    cv::normalize(disparity_bm_gray, disparity_bm_gray_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::normalize(disparity_bm_histeq, disparity_bm_histeq_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::normalize(disparity_bm_blur, disparity_bm_blur_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
    
    cv::Mat bm_gray_flip, bm_histeq_flip, bm_blur_flip;
    cv::flip(disparity_bm_gray_norm, bm_gray_flip, 0);
    cv::flip(disparity_bm_histeq_norm, bm_histeq_flip, 0);
    cv::flip(disparity_bm_blur_norm, bm_blur_flip, 0);
    
    cv::imwrite("data/comparison/7_disparity_bm_gray.png", bm_gray_flip);
    cv::imwrite("data/comparison/8_disparity_bm_histeq.png", bm_histeq_flip);
    cv::imwrite("data/comparison/9_disparity_bm_blur.png", bm_blur_flip);

    // APPROACH 3: StereoSGBM method
    cv::Ptr<cv::StereoSGBM> stereoSGBM = cv::StereoSGBM::create(
        0,                   // minDisparity
        numDisparities,      // numDisparities (multiple of 16)
        blockSize,           // SADWindowSize
        8*blockSize*blockSize, // P1 (penalty parameter)
        32*blockSize*blockSize, // P2
        1,                   // disp12MaxDiff
        63,                  // preFilterCap
        10,                  // uniquenessRatio
        100,                 // speckleWindowSize
        32,                  // speckleRange
        cv::StereoSGBM::MODE_SGBM_3WAY
    );
    
    // Version 3a: StereoSGBM on grayscale only
    cv::Mat disparity_sgbm_gray;
    stereoSGBM->compute(gray_left_orig, gray_right_orig, disparity_sgbm_gray);
    
    // Version 3b: StereoSGBM on histeq images
    cv::Mat disparity_sgbm_histeq;
    stereoSGBM->compute(gray_left, gray_right, disparity_sgbm_histeq);
    
    // Version 3c: StereoSGBM on blurred images
    cv::Mat disparity_sgbm_blur;
    stereoSGBM->compute(blur_left, blur_right, disparity_sgbm_blur);
    
    // Save all StereoSGBM results
    cv::Mat disparity_sgbm_gray_norm, disparity_sgbm_histeq_norm, disparity_sgbm_blur_norm;
    cv::normalize(disparity_sgbm_gray, disparity_sgbm_gray_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::normalize(disparity_sgbm_histeq, disparity_sgbm_histeq_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::normalize(disparity_sgbm_blur, disparity_sgbm_blur_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
    
    cv::Mat sgbm_gray_flip, sgbm_histeq_flip, sgbm_blur_flip;
    cv::flip(disparity_sgbm_gray_norm, sgbm_gray_flip, 0);
    cv::flip(disparity_sgbm_histeq_norm, sgbm_histeq_flip, 0);
    cv::flip(disparity_sgbm_blur_norm, sgbm_blur_flip, 0);
    
    cv::imwrite("data/comparison/10_disparity_sgbm_gray.png", sgbm_gray_flip);
    cv::imwrite("data/comparison/11_disparity_sgbm_histeq.png", sgbm_histeq_flip);
    cv::imwrite("data/comparison/12_disparity_sgbm_blur.png", sgbm_blur_flip);

    // Also save original images for reference (like in the original code)
    cv::imwrite("data/stereo.png", stereo_flip);
    cv::imwrite("data/left_img.png", left_img_flip);
    cv::imwrite("data/right_img.png", right_img_flip);
    
    // Set the final disparity map to the one you prefer
    // For now, we'll use the SGBM with blur as default (you can change this)
    disparity_map = disparity_sgbm_blur_norm.clone();
}
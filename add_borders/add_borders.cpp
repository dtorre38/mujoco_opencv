// Converted code from a stackeroverflow post to C++, thanks Miki!
// https://stackoverflow.com/questions/50484433/what-image-encoding-does-aruco-expect

#include <opencv2/opencv.hpp>
#include <iostream>


void drawBorders(cv::Mat& image) {
    int width = image.cols;
    int height = image.rows;

    // Parameters for the border
    int nTiles = 8;            // Number of tiles
    int padding = 3;           // Padding in tiles
    int cornerLength = 4;      // Length of the corner in tiles
    int dx = width / nTiles;   // Width of each tile
    int dy = height / nTiles;  // Height of each tile

    printf("height = %d; width = %d \n", height, width);
    printf("dx = %d; dy = %d \n", dx, dy);
    
    // Create a new image with padding
    cv::Mat canvas(height + 2 * padding * dy, width + 2 * padding * dx, image.type(), cv::Scalar(255, 255, 255));

    // Draw corners
    cv::rectangle(canvas, cv::Rect(0, 0, dx, dy * cornerLength), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(0, 0, cornerLength * dx, dy), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(canvas.cols - dx, canvas.rows - dy * cornerLength, dx, dy * cornerLength), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(canvas.cols - cornerLength * dx, canvas.rows - dy, cornerLength * dx, dy), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(canvas.cols - dx, 0, dx, dy * cornerLength), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(canvas.cols - cornerLength * dx, 0, cornerLength * dx, dy), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(0, canvas.rows - dy * cornerLength, dx, dy * cornerLength), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(0, canvas.rows - dy, cornerLength * dx, dy), cv::Scalar(0, 0, 0), cv::FILLED);

    // Draw the image on the new canvas
    cv::Rect roi(padding * dx, padding * dy, width, height);
    image.copyTo(canvas(roi));

    // Update the original image to be the new image with borders
    image = canvas;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <image path>" << std::endl;
        return -1;
    }

    std::string imagePath = argv[1];
    cv::Mat image = cv::imread(imagePath);

    if (image.empty()) {
        std::cerr << "Error: Could not load image " << imagePath << std::endl;
        return -1;
    }

    drawBorders(image);

    // std::string outputImagePath = "output_with_borders.png";
    std::string outputImagePath = "marker23.png";
    cv::imwrite(outputImagePath, image);

    std::cout << "Output image saved as " << outputImagePath << std::endl;

    return 0;
}

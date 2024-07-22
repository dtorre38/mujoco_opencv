#include <mujoco/mujoco.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring> // For memcpy
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <mutex>

using namespace std;

mutex mtx;

#define BLOCK_SIZE 16
#define SEARCH_BLOCK_SIZE 50

void disparity();
void compute_disparity(int start_chunk_row, int end_chunk_row, int start_chunk_col, int end_chunk_col, cv::Mat *left_img, cv::Mat *right_img, cv::Mat *disparity_map);
int compare_blocks(const int row, const int col, const int width, const int height, const cv::Mat *left_img, const cv::Mat *right_img);
vector<int> get_chunk_indices(int max, int num_chunks);

// g++ $(pkg-config --cflags --libs opencv4) -std=c++11 disparity.cpp -o disparity

void disparity() {

    // Load images using imread
    cv::Mat stereo = cv::imread("stereo.png", cv::IMREAD_COLOR);
    cv::Mat left = cv::imread("left.png", cv::IMREAD_COLOR);
    cv::Mat right = cv::imread("right.png", cv::IMREAD_COLOR);

    cv::Mat disparity_map;

    // cv::Mat stereo;
    // cv::flip(result.rgbFrame, stereo, 0);
    // cv::imwrite("stereo.png", stereo);

    // cv::Mat frame_rgb = result.rgbFrame;
    // cv::Mat left = frame_rgb(cv::Rect(0, 0, 320, 480));
    // cv::Mat right = frame_rgb(cv::Rect(320, 0, 320, 480));

    // cv::flip(left, left, 0);
    // cv::imwrite("left.png", left);
    // cv::flip(right, right, 0);
    // cv::imwrite("right.png", right);

    // compute disparity map
    static const int num_threads = 8;
    std::vector<int> height_chunks = get_chunk_indices(left.rows, num_threads);
    for (int i = 0; i < height_chunks.size() - 1; ++i) {
        std::thread t(compute_disparity, height_chunks[i], height_chunks[i + 1], 0, left.cols - 1, &left, &right, &disparity_map);
        t.join();
    }

    cv::imwrite("disparitymap.png", disparity_map);
}

int compare_blocks(const int row, const int col, const int width, const int height, const cv::Mat *left_img, const cv::Mat *right_img) {
    int sad = 0;
    int min_col = col;
    int bottom_row = min(row + BLOCK_SIZE, height - 1);
    int bottom_col = min(col + BLOCK_SIZE, width - 1);
    int col_min = max(0, col - SEARCH_BLOCK_SIZE);
    int col_max = min(width, col + SEARCH_BLOCK_SIZE);
    bool first_block = true;
    int min_sad = 0;
    for (int r_indx = col_min; r_indx < col_max; ++r_indx) {
        sad = 0;
        for (int i = row; i < bottom_row; ++i) {
            int r_img_col = r_indx;
            for (int j = col; j < bottom_col; ++j) {
                cv::Scalar left_pixel = left_img->at<uchar>(i, j);
                cv::Scalar right_pixel = right_img->at<uchar>(i, r_img_col);
                sad += abs(left_pixel.val[0] - right_pixel.val[0]);
                ++r_img_col;
            }
        } 

        if (first_block) {
            min_sad = sad;
            min_col = r_indx;
            first_block = false;
        } else {
            if (sad < min_sad) {
                min_sad = sad;
                min_col = r_indx;
            }
        }
    }

    return col - min_col;
}

void compute_disparity(int start_chunk_row, int end_chunk_row, int start_chunk_col, int end_chunk_col, cv::Mat *left_img, cv::Mat *right_img, cv::Mat *disparity_map) {
    int height = left_img->rows;
    int width = left_img->cols;
    for (int i = start_chunk_row; i < end_chunk_row; ++i) {
        for (int j = start_chunk_col; j < end_chunk_col; ++j) {
            int disp = compare_blocks(i, j, height, width, left_img, right_img); 
            if (disp < 0) {
                mtx.lock();
                disparity_map->at<uchar>(i, j) = 0;
                mtx.unlock();
            } else {
                mtx.lock();
                disparity_map->at<uchar>(i, j) = disp;
                mtx.unlock();
            }
        } 
    }
}

vector<int> get_chunk_indices(int max, int num_chunks) {
    vector<int> chunks;
    int step = max / num_chunks;
    for (int i = 0; i < max; i = i + step) {
        chunks.push_back(i);
    }
    chunks[chunks.size() - 1] = max - 1;

    return chunks;
}


int main() {
    disparity();

    return 0;
}

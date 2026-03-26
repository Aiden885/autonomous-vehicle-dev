#include "img_process.hpp"



namespace perception{
/**
 * @brief Loads an image from file into a cv::Mat
 * @param filename Path to the image file to load
 * @param clr_image Output color image matrix
 * @return true if successful, false if failed to load
 */
bool loadImage(const char *filename, cv::Mat &clr_image)
{
  // load the image
    clr_image = cv::imread(filename);
    if (clr_image.empty()) {
        std::cerr << "Could not open or find the image" << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief Saves an image to file
 * @param input Image matrix to save
 * @param filename Output filename to save to
 * @return true if successful, false if failed
 */
bool saveImage(cv::Mat input, const char *filename)
{
     if (input.empty()) {
        std::cerr << "Could not open or find the image" << std::endl;
        return false;
    }
    cv::imwrite(filename, input);
    return true;
}

/**
 * @brief Displays an image in a window
 * @param input Image to display
 * @param window_name Name of the window
 */
void imShow(cv::Mat input, const char *window_name)
{
    cv::imshow(window_name, input);
    cv::waitKey(0);
}

/**
 * @brief Converts a color image to grayscale
 * @param input Input color image
 * @param output Output grayscale image
 * @return true if successful, false if failed
 */
bool imGray(cv::Mat input, cv::Mat &output)
{
    if(input.empty()) {
        std::cerr << "Error: Loading image" << std::endl;
        return false;
    }

    // 将彩色图像转换为灰度图像
    cv::cvtColor(input, output, cv::COLOR_BGR2GRAY);
 
    return true;
}


/**
 * @brief Applies Laplacian edge detection
 * @param input Input image
 * @param output Output edge detected image
 * @param is_norm Whether to normalize output to 0-255 range
 * @return true if successful, false if failed
 */
bool imLaplaceTrans(cv::Mat input, cv::Mat &output, bool is_norm)
{
    if (input.empty()) {
        std::cout << "Error: Loading image" << std::endl;
        return false;
    }
 
    // 应用拉普拉斯算子
    cv::Laplacian(input, output, CV_64F);
 
    // 归一化拉普拉斯算子的输出
    if(is_norm)
        cv::normalize(output, output, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    return true;
}

/**
 * @brief Applies Scharr edge detection operator to an image
 * @details The Scharr operator is used for edge detection and is more accurate than 
 *          Sobel for edge detection, particularly for edges at 45 degrees.
 *          It computes both x and y derivatives and combines them.
 *
 * @param input Input image (grayscale)
 * @param output Output edge detected image 
 * @param scale Optional scale factor for derivatives (default 1)
 * @param delta Optional value added to results (default 0)
 */
void imScharrTrans(cv::Mat input, cv::Mat &output, double scale, double delta)
{
    // Check if input is empty
    if (input.empty()) {
        std::cerr << "Error: Input image is empty" << std::endl;
        return;
    }

    // Temporary matrices to store gradients
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Calculate gradients in x and y directions using Scharr
    cv::Scharr(input, grad_x, CV_16S, 1, 0, scale, delta, cv::BORDER_DEFAULT);
    cv::Scharr(input, grad_y, CV_16S, 0, 1, scale, delta, cv::BORDER_DEFAULT);

    // Convert gradients to absolute values
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Combine the two gradients
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, output);
}


/**
 * @brief Performs histogram equalization on grayscale image
 * @param input Input image
 * @param output Output equalized image
 * @return true if successful, false if failed
 */
bool imEqualizeHistogram(cv::Mat input, cv::Mat &output)
{
    if (input.empty()) {
        std::cout << "图像加载失败" << std::endl;
        return false;
    }
    if(input.channels() != 1)
    {
        cv::Mat gray;
        imGray(input, gray);
        input = gray;
    }
        
 
    // 应用直方图均衡化
    cv::equalizeHist(input, output);

    return true;
}

/**
 * @brief Applies gamma correction to image
 * @param input Input image
 * @param output Output gamma corrected image
 * @param gamma Gamma correction value
 * @return true if successful, false if failed
 */
bool imGammaCorrection(cv::Mat input, cv::Mat &output, double gamma)
{
    // 确保图像读取成功
    if (input.empty()) {
        std::cerr << "Error: Loading image" << std::endl;
        return false;
    }
 
    // 创建一个伽马表
    cv::Mat gammaTable(1, 256, CV_8U);
 
    // 填充伽马表
    for (int i = 0; i < 256; i++) {
        gammaTable.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i / 255.0, 2.2) * 255.0);
    }

    // 应用伽马矫正
    cv::LUT(input, gammaTable, output);
    return true;
}

/**
 * @brief Inverts image colors
 * @param input Input image
 * @param output Output inverted image
 * @return true if successful, false if failed
 */
bool imInvertTransform(cv::Mat input, cv::Mat &output)
{
    if(input.empty()) {
        std::cout << "图像加载失败" << std::endl;
        return false;
    }

    // 反转图像
    cv::bitwise_not(input, output);
    return true;
}

/**
 * @brief Applies logarithmic transform to image
 * @param input Input image
 * @param output Output transformed image
 * @param c Scaling constant
 * @return true if successful, false if failed
 */
bool imLogTransform(cv::Mat input, cv::Mat &output, double c)
{
    if(input.empty()) {
        std::cout << "图像加载失败" << std::endl;
        return false;
    }   
    // Convert input image to float for processing
    cv::Mat float_img;
    input.convertTo(float_img, CV_32F);
    
    // Add 1 to avoid log(0)
    float_img = float_img + 1.0;
    
    // Apply log transform: output = c * log(1 + input)
    for(int i = 0; i < input.rows; i++) {
        for(int j = 0; j < input.cols; j++) {
            float_img.at<float>(i,j) = c * std::log(float_img.at<float>(i,j));
        }
    }
    
    // Normalize the output to 0-255 range
    cv::normalize(float_img, float_img, 0, 255, cv::NORM_MINMAX);
    
    // Convert back to original type
    float_img.convertTo(output, input.type());

    return true;
}

/**
 * @brief Applies mean filter to image
 * @param input Input image
 * @param output Output filtered image
 * @param kernel_size_x Kernel width
 * @param kernel_size_y Kernel height
 * @return true if successful, false if failed
 */
bool imMeanFilter(cv::Mat input, cv::Mat &output, int kernel_size_x, int kernel_size_y)
{
    // 检查图像是否成功加载
    if(input.empty()) {
        std::cerr << "Error: Loading image" << std::endl;
        return false;
    }

    // 应用均值滤波
    cv::blur(input, output, cv::Size(kernel_size_x, kernel_size_y));

    return true;
}

/**
 * @brief Applies median filter to image
 * @param input Input image
 * @param output Output filtered image
 * @param kernel_size Size of median filter kernel
 * @return true if successful, false if failed
 */
bool imMedianFilter(cv::Mat input, cv::Mat &output, int kernel_size)
{
    // 读取图像
    if (input.empty())
    {
        std::cout << "无法读取图像" << std::endl;
        return false;
    }

    // 进行中值滤波
    cv::medianBlur(input, output, kernel_size);

    return true;
}

/**
 * @brief Applies Gaussian filtering to an image
 * @param input Input source image
 * @param output Output filtered image
 * @param kernel_size Size of Gaussian kernel (must be odd)
 * @param sigma Standard deviation of Gaussian kernel
 */
void imGaussianFilter(cv::Mat input, cv::Mat &output, int kernel_size, double sigma)
{
    if (input.empty()) {
        std::cerr << "Error: Input image is empty" << std::endl;
        return;
    }

    // Ensure kernel size is odd
    if (kernel_size % 2 == 0) {
        kernel_size++;
    }

    // Create Gaussian kernel
    cv::Mat kernel = cv::getGaussianKernel(kernel_size, sigma);
    cv::Mat kernel2D = kernel * kernel.t();

    // Apply filter
    cv::filter2D(input, output, -1, kernel2D);
}


/**
 * @brief Applies sharpening filter to image using unsharp masking
 * @param input Input image
 * @param output Output sharpened image
 * @param sigma Gaussian blur sigma (default 1.0)
 * @param amount Sharpening strength (default 1.5)
 * @param threshold Minimum difference for sharpening (default 0)
 */
void imSharpeningFilter(cv::Mat input, cv::Mat &output, double sigma, double amount, int threshold)
{
    if (input.empty()) {
        std::cerr << "Error: Input image is empty" << std::endl;
        return;
    }

    cv::Mat blurred;
    // Create a Gaussian blurred version
    cv::GaussianBlur(input, blurred, cv::Size(0, 0), sigma);

    // Subtract blurred image from original to get mask
    cv::Mat mask;
    cv::subtract(input, blurred, mask);

    // Add weighted mask back to original
    output = input.clone();
    for(int i = 0; i < input.rows; i++) {
        for(int j = 0; j < input.cols; j++) {
            for(int c = 0; c < input.channels(); c++) {
                int diff = mask.at<cv::Vec3b>(i,j)[c];
                // Only sharpen if difference exceeds threshold
                if(std::abs(diff) > threshold) {
                    int val = input.at<cv::Vec3b>(i,j)[c] + amount * diff;
                    output.at<cv::Vec3b>(i,j)[c] = cv::saturate_cast<uchar>(val);
                }
            }
        }
    }
}


/**
 * @brief Applies bilateral filter to image
 * @param input Input image
 * @param output Output filtered image
 * @param diameter Diameter of pixel neighborhood
 * @param sigma_color Filter sigma in color space
 * @param sigma_space Filter sigma in coordinate space
 * @return true if successful, false if failed
 */
bool imBilateralFilter(cv::Mat input, cv::Mat &output, int diameter, double sigma_color, double sigma_space)
{
    // 读取图像
    if (input.empty())
    {
        std::cout << "无法读取图像" << std::endl;
        return false;
    }

    cv::bilateralFilter(input, output, diameter, sigma_color, sigma_space);

    return true;
}




/**
 * @brief Draws a rectangle on an image
 * @param input Image to draw on
 * @param rect Rectangle coordinates
 * @param color Rectangle color
 * @param width Rectangle line width
 */
void imDrawRectangle (cv::Mat input, cv::Rect rect, cv::Scalar color, int width)
{
  //draw the rectangle
  cv::rectangle(input, cv::Point(rect.x, rect.y),
              cv::Point(rect.x + rect.width-1, rect.y + rect.height-1),
              color, width);

}

/**
 * @brief Draws text on an image
 * @param input Image to draw on
 * @param str Text string to draw
 * @param point Text position
 * @param fontFace Font type
 * @param size Font size
 * @param color Text color
 */
void imDrawText(cv::Mat input, char* str, cv::Point point, int fontFace,
                 float size, cv::Scalar color)
{
  cv::putText(input, str, point, fontFace, size, color);

}

/**
 * @brief Draws a circle on an image
 * @param input Image to draw on
 * @param center Circle center point
 * @param radius Circle radius
 * @param color Circle color
 * @param thickness Line thickness
 * @param lineType Line type
 * @param shift Number of fractional bits
 */
void imDrawCircle(cv::Mat input, cv::Point center, int radius, 
                  cv::Scalar color, int thickness,
                  int lineType, int shift)
{
    cv::circle(input, center, radius, color, thickness, lineType, shift);
}

/**
 * @brief Performs double thresholding on an image
 * @details Applies double thresholding algorithm to segment image into three regions:
 *          strong edges (above high threshold), weak edges (between thresholds),
 *          and non-edges (below low threshold)
 * 
 * @param input Input image
 * @param output Output thresholded image
 * @param low_threshold Lower threshold value (default 20)
 * @param high_threshold Higher threshold value (default 50)
 * @param strong_pixel Strong edge pixel value (default 255)
 * @param weak_pixel Weak edge pixel value (default 128)
 */
void imDoubleThresholding(cv::Mat input, cv::Mat &output, 
                         double low_threshold, double high_threshold,
                         uchar strong_pixel, uchar weak_pixel)
{
    // Create output image same size as input
    output = cv::Mat::zeros(input.size(), CV_8UC1);

    // Apply thresholds
    for(int i = 0; i < input.rows; i++) {
        for(int j = 0; j < input.cols; j++) {
            double pixel = input.at<uchar>(i,j);
            
            if(pixel >= high_threshold) {
                // Strong edge
                output.at<uchar>(i,j) = strong_pixel;
            }
            else if(pixel >= low_threshold && pixel < high_threshold) {
                // Weak edge
                output.at<uchar>(i,j) = weak_pixel;
            }
            else {
                // Non-edge
                output.at<uchar>(i,j) = 0;
            }
        }
    }

    // Hysteresis - convert weak edges to strong if connected to strong edge
    cv::Mat temp = output.clone();
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

    for(int i = 1; i < output.rows-1; i++) {
        for(int j = 1; j < output.cols-1; j++) {
            if(temp.at<uchar>(i,j) == weak_pixel) {
                // Check 8-connected neighbors
                bool has_strong_neighbor = false;
                for(int k = 0; k < 8; k++) {
                    if(temp.at<uchar>(i+dx[k], j+dy[k]) == strong_pixel) {
                        has_strong_neighbor = true;
                        break;
                    }
                }
                // Convert weak to strong if connected to strong edge
                output.at<uchar>(i,j) = has_strong_neighbor ? strong_pixel : 0;
            }
        }
    }
}


/**
 * @brief Applies Roberts edge detection
 * @param input Input image
 * @param output Output edge detected image
 */
void imRoberts(cv::Mat input, cv::Mat &output)
{
    cv::Mat image_bw1, image_bw2;
    //构建检测核
	cv::Mat kernel1 = (cv::Mat_<float>(2, 2) << -1, 0, 0, 1);
	cv::Mat kernel2 = (cv::Mat_<float>(2, 2) << 0, -1, 1, 0);
	//利用filter2D进行处理
	cv::filter2D(input, image_bw1, -1, kernel1);
	cv::filter2D(input, image_bw2, -1, kernel2);
	//结果取绝对值
	cv::convertScaleAbs(image_bw1, image_bw1);
	cv::convertScaleAbs(image_bw2, image_bw2);
	//转换为二值图
	cv::threshold(image_bw1, image_bw1, 30, 255, 0);
	cv::threshold(image_bw2, image_bw2, 30, 255, 0);
	//两个方向的结果相加
	output = image_bw1 + image_bw2;
}

/**
 * @brief Applies Prewitt edge detection
 * @param input Input image
 * @param output Output edge detected image
 */
void imPrewitt(cv::Mat input, cv::Mat &output)
{
    // 创建输出图像
    cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y;
 
    // 定义Prewitt算子的卷积核
    cv::Mat kernel_x = (cv::Mat_<double>(3, 3) << -1, 0, 1, -1, 0, 1, -1, 0, 1);
    cv::Mat kernel_y = (cv::Mat_<double>(3, 3) << 1, 1, 1, 0, 0, 0, -1, -1, -1);
 
    // 对图像应用Prewitt算子
    cv::filter2D(input, grad_x, CV_64F, kernel_x);
    cv::filter2D(input, grad_y, CV_64F, kernel_y);
 
    // 转换为8位无符号整型
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::convertScaleAbs(grad_y, abs_grad_y);
 
    // 合并梯度图像
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, output);
}

/**
 * @brief Applies morphological dilation
 * @param input Input image
 * @param output Output dilated image
 * @param kernel_size Size of structuring element
 */
void imMorphologyDilation(cv::Mat input, cv::Mat &output, int kernel_size)
{
    // Create structuring element
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    
    // Apply dilation
    cv::dilate(input, output, kernel);
}

/**
 * @brief Applies morphological erosion
 * @param input Input image
 * @param output Output eroded image
 * @param kernel_size Size of structuring element
 */
void imMorphologyErosion(cv::Mat input, cv::Mat &output, int kernel_size)
{
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                        cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
                                        cv::Point(-1, -1));
    cv::erode(input, output, element,cv::Point(-1,-1),1);
}

/**
 * @brief Applies morphological opening
 * @param input Input image
 * @param output Output opened image
 * @param kernel_size Size of structuring element
 */
void imMorphologyOpen(cv::Mat input, cv::Mat &output, int kernel_size)
{
    // Create structuring element
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                                             cv::Size(kernel_size, kernel_size));
    
    // Apply morphological opening (erosion followed by dilation)
    cv::morphologyEx(input, output, cv::MORPH_OPEN, kernel);
}

/**
 * @brief Applies morphological closing
 * @param input Input image
 * @param output Output closed image
 * @param kernel_size Size of structuring element
 */
void imMorphologyClose(cv::Mat input, cv::Mat &output, int kernel_size)
{
    // Create structuring element
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                             cv::Size(kernel_size, kernel_size));
    
    // Apply morphological closing (dilation followed by erosion)
    cv::morphologyEx(input, output, cv::MORPH_CLOSE, kernel);
}

/**
 * @brief Performs nearest neighbor interpolation
 * @param input Input image
 * @param output Output interpolated image
 * @param sx Scale factor in x direction
 * @param sy Scale factor in y direction
 */
void imNearestInterpolation(cv::Mat input, cv::Mat &output, double sx, double sy)
{
    int output_cols = round(input.cols * sx);  // 列  ==x
	int output_rows = round(input.rows * sy);  // 行==y
 
 
	output = cv::Mat(output_rows, output_cols, input.type());
 
	//灰度图像处理
	if (input.channels() == 1)
	{
		for (int i = 0; i < output.rows; i++)
		{
			for (int j = 0; j < output.cols; j++)
			{
				//插值计算，取最近值插入到新的图像中
				int i_new = round(i / sy);
				int j_new = round(j / sx);
 
				if (i_new > input.rows - 1)
				{
					i_new = input.rows - 1;
				}
				if (j_new > input.cols - 1)
				{
					j_new = input.cols - 1;
				}
				output.at<uchar>(i, j) = input.at<uchar>(i_new, j_new);
			}
		}
	}
	//彩色图像处理
	else {
		for (int i = 0; i < output.rows; i++)
		{
			for (int j = 0; j < output.cols; j++)
			{
				int i_new = round(i / sy);
				int j_new = round(j / sx);
				if (i_new > input.rows - 1)
				{
					i_new = input.rows - 1;
				}
				if (j_new > input.cols - 1)
				{
					j_new = input.cols - 1;
				}
 
				//B
				output.at<cv::Vec3b>(i, j)[0] = input.at<cv::Vec3b>(i_new, j_new)[0];
				//G
				output.at<cv::Vec3b>(i, j)[1] = input.at<cv::Vec3b>(i_new, j_new)[1];
				//R
				output.at<cv::Vec3b>(i, j)[2] = input.at<cv::Vec3b>(i_new, j_new)[2];
			}
		}
	}
}

/**
 * @brief Performs bilinear interpolation
 * @param input Input image
 * @param output Output interpolated image
 * @param scale_x Scale factor in x direction
 * @param scale_y Scale factor in y direction
 */
void imBiLineInterpolate(cv::Mat input, cv::Mat &output, double scale_x,double scale_y)
{
        
    int result_H = static_cast<int>(input.rows * scale_y);
	int result_W = static_cast<int>(input.cols * scale_x);
 
	output = cv::Mat::zeros(cv::Size(result_W, result_H), input.type());

	for (int i = 0; i < output.rows; i++)
	{
		for (int j = 0; j < output.cols; j++)
		{
			// 非常重要的一步就是用新的图像来推算出原来图像的四个像素的位置和灰度值
			double before_x = double(j + 0.5) / scale_x - 0.5f;
			double before_y = double(i + 0.5) / scale_y - 0.5;
 
			int top_y = static_cast<int>(before_y);
			int bottom_y = top_y + 1;
			int left_x = static_cast<int>(before_x);
			int right_x = left_x + 1;
 
 
			//计算变换前坐标的小数部分
			double u = before_x - left_x;
			double v = before_y - top_y;
 
			// 如果计算的原始图像的像素大于真实原始图像尺寸
			if ((top_y >= input.rows - 1) && (left_x >= input.cols - 1))
			{
				//右下角
				output.at<uchar>(i, j) = (1. - u) * (1. - v) * input.at<uchar>(top_y, left_x);
			}
			else if (top_y >= input.rows - 1)
			{
				//最后一行
 
                output.at<uchar>(i, j)
                    = (1. - u) * (1. - v) * input.at<uchar>(top_y, left_x)
                + (1. - v) * u * input.at<uchar>(top_y, right_x);
	
			}
			else if (left_x >= input.cols - 1)
			{
                output.at<uchar>(i, j)
                    = (1. - u) * (1. - v) * input.at<uchar>(top_y, left_x)
                    + (v) * (1. - u) * input.at<uchar>(bottom_y, left_x);
			}
			else
			{
                output.at<uchar>(i, j)
                    = (1. - u) * (1. - v) * input.at<uchar>(top_y, left_x)
                    + (1. - v) * (u)*input.at<uchar>(top_y, right_x)
                    + (v) * (1. - u) * input.at<uchar>(bottom_y, left_x)
                    + (u) * (v)*input.at<uchar>(bottom_y, right_x);
			}
 
		}
	}  
}




/**
 * @brief Performs bicubic interpolation
 * @param input Input image
 * @param output Output interpolated image
 * @param scale_x Scale factor in x direction
 * @param scale_y Scale factor in y direction
 */
void imCubicInterpolation(cv::Mat input, cv::Mat &output, double scale_x,double scale_y)
{
    // Create output image
    output.create(cvRound(input.rows * scale_y), cvRound(input.cols * scale_x), input.type());

    // For each pixel in destination image
    for (int i = 0; i < output.rows; i++) {
        for (int j = 0; j < output.cols; j++) {
            // Calculate corresponding position in source image
            double input_x = j / scale_x;
            double input_y = i / scale_y;

            // Get integer and fractional parts
            int x = (int)input_x;
            int y = (int)input_y;
            double dx = input_x - x;
            double dy = input_y - y;

            // Cubic interpolation kernel
            auto cubic = [](double x) {
                x = std::abs(x);
                if (x <= 1)
                    return 1.5*x*x*x - 2.5*x*x + 1.0;
                else if (x < 2)
                    return -0.5*x*x*x + 2.5*x*x - 4.0*x + 2.0;
                return 0.0;
            };

            // Calculate weights for 16 surrounding pixels
            double weights[4][4];
            for (int m = 0; m < 4; m++) {
                for (int n = 0; n < 4; n++) {
                    weights[m][n] = cubic(m - 1 - dy) * cubic(n - 1 - dx);
                }
            }

            // Calculate weighted sum
            double sum = 0;
            double weightSum = 0;
            
            for (int m = 0; m < 4; m++) {
                for (int n = 0; n < 4; n++) {
                    int cur_y = y + m - 1;
                    int cur_x = x + n - 1;
                    
                    // Check bounds
                    if (cur_x >= 0 && cur_x < input.cols && cur_y >= 0 && cur_y < input.rows) {
                        sum += input.at<uchar>(cur_y, cur_x) * weights[m][n];
                        weightSum += weights[m][n];
                    }
                }
            }

            // Normalize and set output pixel
            if (weightSum != 0)
                output.at<uchar>(i, j) = cv::saturate_cast<uchar>(sum / weightSum);
            else
                output.at<uchar>(i, j) = 0;
        }
    }

}

/**
 * @brief Gets nearest neighbor pixel value
 * @param input Input image
 * @param result Output pixel value
 * @param pos_x X coordinate
 * @param pos_y Y coordinate
 */
void imGetNearestNeighbor(cv::Mat input, uchar& result, double pos_x,double pos_y)
{
    int x = static_cast<int>(pos_x);
    if(pos_x - x > 0.5)
        x += 1;
    int y = static_cast<int>(pos_y);
    if(pos_y - y > 0.5)
        y += 1;
    result = input.at<uchar>(x, y);  
}

/**
 * @brief Gets bilinearly interpolated pixel value
 * @param input Input image
 * @param result Output pixel value
 * @param pos_x X coordinate
 * @param pos_y Y coordinate
 */
void imGetBilinearInterpolation(cv::Mat input, uchar& result, double pos_x,double pos_y)
{
    int x = static_cast<int>(pos_x);
    int y = static_cast<int>(pos_y);
    double dx = pos_x - x;
    double dy = pos_y - y;
    result = input.at<uchar>(x, y) * (1 - dx) * (1 - dy) + input.at<uchar>(x + 1, y) * dx * (1 - dy) + \
             input.at<uchar>(x, y + 1) * (1 - dx) * dy + input.at<uchar>(x + 1, y + 1) * dx * dy;
}

/**
 * @brief Finds contours in a binary image
 * @details Uses OpenCV's findContours function to detect contours in a binary image.
 *          The input image should be binary (thresholded). The function detects edges
 *          and outputs contours as vector of points.
 *
 * @param input Input binary image
 * @param contours Output vector of detected contours, where each contour is a vector of points
 * @param mode Contour retrieval mode (default: CV_RETR_EXTERNAL)
 * @param method Contour approximation method (default: CV_CHAIN_APPROX_SIMPLE)
 */
void imGetContours(cv::Mat input, cv::Mat &output, std::vector<std::vector<cv::Point>> &contours,
                  int mode,
                  int method)
{
    // Check if input is empty
    if (input.empty()) {
        std::cerr << "Error: Input image is empty" << std::endl;
        return;
    }

    // Create a copy of input image
    cv::Mat image = input.clone();

    // Clear output contours vector
    contours.clear();

    // Find contours
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, mode, method);

    drawContours(output, contours, -1, cv::Scalar(255, 0, 255), 2);

    // vector<vector<Point>>conPoly(contours.size());
	// vector<Rect>boundRect(contours.size());

    // //排除干扰
	// for (int i = 0;i < contours.size();i++) {
	// 	//计算轮廓面积 
	// 	int area = contourArea(contours[i]);
	// 	string objectType;
	// 	cout << area << endl;
	// 	if (area > 1000) {
	// 		//arcLength(contours[i], true);计算轮廓周长  
	// 		//InputArray类型的curve，输入的向量，二维点（轮廓顶点），可以为std::vector或Mat类型。
	// 		//bool类型的closed，用于指示曲线是否封闭的标识符，一般设置为true。
	// 		float peri = arcLength(contours[i], true);
	// 		//对图像轮廓点进行多边形拟合
	// 		approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
	// 		cout << conPoly[i].size() << endl;
	// 		// 表示返回矩形边界左上角顶点的坐标值及矩形边界的宽和高
	// 		boundRect[i] = boundingRect(conPoly[i]);
	// 		//得到特征点
	// 		int objCor = (int)conPoly[i].size();
	// 		//判断图像
	// 		if (objCor == 3) {
	// 			objectType = "Tri";
	// 		}
	// 		if (objCor == 4) {
	// 			float aspRatio = (float)boundRect[i].width / (float)boundRect[i].height;
	// 			cout << aspRatio << endl;
	// 			if (aspRatio > 0.95 && aspRatio < 1.05) {
	// 				objectType = "Square";
	// 			}
	// 			else objectType = "Rect";
	// 		}
	// 		if (objCor > 4) {
	// 			objectType = "Circle";
	// 		}
 
	// 	}
	// }
}
}
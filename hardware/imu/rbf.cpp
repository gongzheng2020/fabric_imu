#include "rbf.h"
#include <cmath>

RBF::RBF(int input_dim, int hidden_dim, int output_dim)
    : input_dim(input_dim), hidden_dim(hidden_dim), output_dim(output_dim) {
    centers = new float*[hidden_dim];
    centers_velocity = new float*[hidden_dim];
    for (int i = 0; i < hidden_dim; ++i) {
        centers[i] = new float[input_dim]();
        centers_velocity[i] = new float[input_dim]();
    }

    widths = new float[hidden_dim]();
    widths_velocity = new float[hidden_dim]();

    weights = new float*[hidden_dim];
    weights_velocity = new float*[hidden_dim];
    for (int i = 0; i < hidden_dim; ++i) {
        weights[i] = new float[output_dim]();
        weights_velocity[i] = new float[output_dim]();
    }
}

RBF::~RBF() {
    for (int i = 0; i < hidden_dim; ++i) {
        delete[] centers[i];
        delete[] centers_velocity[i];
        delete[] weights[i];
        delete[] weights_velocity[i];
    }
    delete[] centers;
    delete[] centers_velocity;
    delete[] widths;
    delete[] widths_velocity;
    delete[] weights;
    delete[] weights_velocity;
}

float RBF::gaussian(float* input, float* center, float width) {
    float sum = 0.0f;
    for (int i = 0; i < input_dim; ++i) {
        float diff = input[i] - center[i];
        sum += diff * diff;
    }
    return exp(-sum / (2 * width * width));
}

float* RBF::rbf_update(float* input, float* target, float learning_rate, float momentum) {
    // 计算隐藏层输出
    float* hidden_outputs = new float[hidden_dim];
    for (int j = 0; j < hidden_dim; ++j) {
        hidden_outputs[j] = gaussian(input, centers[j], widths[j]);
    }

    // 计算输出并保存预测结果
    float* outputs = new float[output_dim]();
    for (int k = 0; k < output_dim; ++k) {
        float output = 0.0f;
        for (int j = 0; j < hidden_dim; ++j) {
            output += hidden_outputs[j] * weights[j][k];
        }
        outputs[k] = output;
    }

    // 如果提供了目标值，则更新中心、宽度和权重
    if (target != nullptr) {
        for (int j = 0; j < hidden_dim; ++j) {
            float error_sum = 0.0f;
            for (int k = 0; k < output_dim; ++k) {
                float error = target[k] - outputs[k];
                error_sum += error * weights[j][k];
            }

            // 更新中心
            for (int i = 0; i < input_dim; ++i) {
                float delta_center = learning_rate * error_sum * hidden_outputs[j] * (input[i] - centers[j][i]) / (widths[j] * widths[j]);
                centers_velocity[j][i] = momentum * centers_velocity[j][i] + delta_center;
                centers[j][i] += centers_velocity[j][i];
            }

            // 更新宽度
            float delta_width = learning_rate * error_sum * hidden_outputs[j] * (hidden_outputs[j] - 1) / widths[j];
            widths_velocity[j] = momentum * widths_velocity[j] + delta_width;
            widths[j] += widths_velocity[j];

            // 更新权重
            for (int k = 0; k < output_dim; ++k) {
                float error = target[k] - outputs[k];
                float delta_weight = learning_rate * error * hidden_outputs[j];
                weights_velocity[j][k] = momentum * weights_velocity[j][k] + delta_weight;
                weights[j][k] += weights_velocity[j][k];
            }
        }
    }

    delete[] hidden_outputs;
    return outputs;
}

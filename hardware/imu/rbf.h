#ifndef RBF_H
#define RBF_H

class RBF {
public:
    RBF(int input_dim, int hidden_dim, int output_dim);
    ~RBF();

    float* rbf_update(float* input, float* target, float learning_rate, float momentum); // 更新并预测方法，加入动量因子

private:
    int input_dim;
    int hidden_dim;
    int output_dim;

    float** centers; // RBF centers
    float* widths;   // RBF widths
    float** weights; // Output layer weights

    // 动量因子相关变量
    float** centers_velocity;
    float* widths_velocity;
    float** weights_velocity;

    float gaussian(float* input, float* center, float width);
};

#endif // RBF_H

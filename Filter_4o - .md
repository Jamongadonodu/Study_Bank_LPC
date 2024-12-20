
---

# <center><font size=8><b id="back">省流</b></font></center>

<font size=5>

- *实时性要求高且硬件资源有限时*：**[低通](#低通)、[高通](#高通)、[EMA滤波](#指数加权)**。
- *动态估计或传感器融合时*：**[卡尔曼滤波](#卡尔曼)**。
- *对突发性噪声敏感时*：**[中值滤波](#中值)**。
- *特定频率分析时*：**[带通滤波器](#带通)、[傅里叶变换滤波器](#傅里叶)**。
- *音频处理、相位要求时*：**[IIR滤波器](#IIR)、[FIR滤波器](#FIR)**。

</font>

## <a id="低通">低通滤波器（Simple Low-pass Filter）</a>

### 低通滤波器可以平滑输入信号，去除高频噪声。

**工作原理**：
低通滤波器允许低频信号通过，同时衰减高频信号。通常用来平滑输入信号，去除高频噪声。

**优点**：
简单易实现：算法简单，易于实现。
去除高频噪声：适合去除系统中的高频干扰或噪声。
较少的计算开销：对硬件要求低，适合嵌入式系统。

**缺点**：
信号延迟：会引入一定的延迟，特别是在低频信号上表现为相位延迟。
无法消除低频漂移：如果噪声源是低频的，低通滤波器的效果不佳。
无法应对突发性高频噪声：对于突发性的高频噪声，滤波效果不如高通滤波器或其他更加专门的滤波器。

**应用场景**：
温度和压力传感器：传感器输出的信号往往包含高频噪声，低通滤波器能够有效去除。
音频信号处理：用于去除音频中的高频噪声或杂音。
电机控制系统：减少传感器或控制信号中的高频噪声，改善稳定性。

```

#define ALPHA 0.1f  // 调整滤波强度
float prevOutput = 0.0f;

float LowPassFilter(float input)
{
    prevOutput = ALPHA * input + (1 - ALPHA) * prevOutput;
    return prevOutput;
}

```
 
[<font size=5> back </font>](#back) 

---

## <a id="高通">高通滤波器（High-pass Filter）</a>

### 高通滤波器去除低频噪声，保留信号中的快速变化。

**工作原理**：
低通滤波器允许低频信号通过，同时衰减高频信号。通常用来平滑输入信号，去除高频噪声。

**优点**：
简单易实现：算法简单，易于实现。
去除高频噪声：适合去除系统中的高频干扰或噪声。
较少的计算开销：对硬件要求低，适合嵌入式系统。

**缺点**：
信号延迟：会引入一定的延迟，特别是在低频信号上表现为相位延迟。
无法消除低频漂移：如果噪声源是低频的，低通滤波器的效果不佳。
无法应对突发性高频噪声：对于突发性的高频噪声，滤波效果不如高通滤波器或其他更加专门的滤波器。

**应用场景**：
温度和压力传感器：传感器输出的信号往往包含高频噪声，低通滤波器能够有效去除。
音频信号处理：用于去除音频中的高频噪声或杂音。
电机控制系统：减少传感器或控制信号中的高频噪声，改善稳定性。

```

#define ALPHA 0.1f  // 调整滤波强度
float prevInput = 0.0f;
float prevOutput = 0.0f;

float HighPassFilter(float input)
{
    float output = ALPHA * (prevOutput + input - prevInput);
    prevInput = input;
    prevOutput = output;
    return output;
}

```

[<font size=5> back </font>](#back) 

---

## <a id="带通">带通滤波器（Band-pass Filter）</a>

### 带通滤波器允许一定频率范围的信号通过。

**工作原理**：
带通滤波器仅允许某一频率范围内的信号通过，抑制其他频率的信号。它结合了低通和高通滤波器的特性。

**优点**：
精确选择频率范围：适用于提取特定频段的信号，能够有效过滤掉不需要的频率。
减小高频和低频噪声的干扰：有效去除带外频率的干扰
。
**缺点**：
计算开销较大：带通滤波器通常需要更复杂的算法实现，计算量比低通和高通滤波器更大。
对带宽的选择敏感：频带的宽度和中心频率的选择对滤波效果有重要影响，若选择不当可能会影响滤波效果。

**应用场景**：
通信系统：调频（FM）或调幅（AM）广播信号的带通滤波。
无线传感器网络：传感器信号提取时去除杂波。
心电图（ECG）：通过带通滤波器选择心电信号特定频率范围（通常是0.5Hz到50Hz之间）。

```
float BandPassFilter(float input, float *prevInput, float *prevOutput)
{
    float output = (input - *prevInput) - *prevOutput;
    *prevInput = input;
    *prevOutput = output;
    return output;
}

```

[<font size=5> back </font>](#back) 

---

## <a id="卡尔曼">卡尔曼滤波器（Kalman Filter）</a>

### 卡尔曼滤波器用于动态系统中，提供优化的估计。

**工作原理**：
卡尔曼滤波器是一种递归的最优估计算法，基于系统的动态模型（预测）和测量数据（更新）来估计系统状态。它适用于线性系统的状态估计问题。

**优点**：
最优估计：能对带有噪声的信号进行最佳估计，特别适用于动态系统。
动态调整：卡尔曼滤波器不仅可以滤除噪声，还能随着时间的推移更新对系统的估计。
能处理多变量问题：能够处理多个传感器的数据融合问题。

**缺点**：
复杂的数学模型：需要了解系统的动态方程和噪声模型，设计和实现复杂。
对系统的线性假设有限制：卡尔曼滤波器对线性系统最有效，对于非线性系统需要扩展（如扩展卡尔曼滤波器）。

**应用场景**：
无人驾驶与机器人定位：用于GPS和IMU传感器数据融合。
航天与航空：用于航天器或飞机的导航系统。
自动驾驶系统：车辆的定位、速度估计。

```

typedef struct {
    float Q;  // 过程噪声协方差
    float R;  // 测量噪声协方差
    float X;  // 状态估计
    float P;  // 估计误差协方差
    float K;  // 卡尔曼增益
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf, float Q, float R, float initialEstimate, float initialErrorCovariance)
{
    kf->Q = Q;
    kf->R = R;
    kf->X = initialEstimate;
    kf->P = initialErrorCovariance;
}

float Kalman_Filter(KalmanFilter *kf, float measurement)
{
    // 预测步骤
    kf->P += kf->Q;
    
    // 更新步骤
    kf->K = kf->P / (kf->P + kf->R);
    kf->X += kf->K * (measurement - kf->X);
    kf->P *= (1 - kf->K);

    return kf->X;
}

```

[<font size=5> back </font>](#back) 

---

## <a id="中值">中值滤波器（Median Filter）</a>

### 中值滤波器通过取窗口内的中值来减少噪声。

**工作原理**：
中值滤波器通过取邻域内数据的中值来替代当前值，能够有效消除图像或信号中的脉冲噪声（如椒盐噪声）。

**优点**：
去除尖刺噪声：对于突发的脉冲噪声非常有效。
不引入过多延迟：由于其本身不依赖于前后数据，滤波结果相对平滑。

**缺点**：
处理慢变信号时效果较差：对于平滑信号的处理不如低通滤波器。
对信号的平滑效果不如均值滤波：中值滤波在信号的平滑效果上可能逊色于均值滤波。

**应用场景**：
图像处理：图像去噪，尤其是处理带有椒盐噪声的图像。
音频信号处理：去除音频中的突发噪声。
传感器数据清洗：清理由于硬件问题产生的瞬时信号异常。

```

#define WINDOW_SIZE 5
float window[WINDOW_SIZE];

float MedianFilter(float input)
{
    // 将输入加入窗口并排序
    for (int i = WINDOW_SIZE - 1; i > 0; i--) {
        window[i] = window[i-1];
    }
    window[0] = input;

    // 对窗口排序
    float temp;
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
        for (int j = i + 1; j < WINDOW_SIZE; j++) {
            if (window[i] > window[j]) {
                temp = window[i];
                window[i] = window[j];
                window[j] = temp;
            }
        }
    }

    return window[WINDOW_SIZE / 2]; // 返回中值
}

```

[<font size=5> back </font>](#back) 

---

## <a id="指数加权">指数加权移动平均滤波器（Exponential Moving Average Filter）</a>

### 此滤波器可以平滑时间序列数据，保留较大权重的最新数据。

**工作原理**：
EMA滤波器是一种加权移动平均滤波器，其中最新数据点的权重较大，过去数据点的权重按指数衰减。

**优点**：
实现简单：仅需要存储最近的输入值及计算加权平均。
适应性强：响应迅速，适合实时信号处理。

**缺点**：
对剧烈变化的信号跟踪能力弱：当信号突变时，EMA可能响应过慢。
无法消除长期趋势：在数据有显著长期变化时，EMA可能不足以滤除趋势。

**应用场景**：
股票市场分析：用于股市中的短期趋势分析。
传感器数据平滑：用于平滑气象传感器、工业传感器等数据。
网络流量监测：实时跟踪带宽或流量波动。

```

#define ALPHA 0.1f  // 平滑系数
float prevOutput = 0.0f;

float ExponentialMovingAverageFilter(float input)
{
    prevOutput = ALPHA * input + (1 - ALPHA) * prevOutput;
    return prevOutput;
}

```

[<font size=5> back </font>](#back) 

---

## <a id="傅里叶">傅里叶变换滤波（Fourier Transform Filtering）</a>

### 使用傅里叶变换进行频域滤波。

**工作原理**：
傅里叶变换滤波器将信号从时域转换到频域，进行频谱分析，去除不需要的频率成分，再通过逆傅里叶变换恢复信号。

**优点**：
精确频率选择：可以精确地过滤掉特定频率范围内的信号。
适用于周期性信号：能够分辨周期性信号的频率成分。

**缺点**：
计算量大：需要进行频域转换和逆变换，计算开销大，实时性差。
不适用于非平稳信号：对信号的时域特性较弱，难以处理非平稳信号

**应用场景**：
音频信号处理：音乐或语音信号的频率分析和噪声滤除。
地震波分析：地震信号频谱分析。
通信系统：频谱分析，带宽控制。

```

#include <math.h>

void FourierFilter(float *data, int length, float low_cutoff, float high_cutoff)
{
    for (int i = 0; i < length; i++) {
        float freq = i * (sampling_rate / length);
        if (freq < low_cutoff || freq > high_cutoff) {
            data[i] = 0;  // 阻止频率范围外的数据
        }
    }
}

```

[<font size=5> back </font>](#back) 

---

## <a id="双向">双向滤波器（Bidirectional Filter）</a>

### 结合前向和后向滤波的效果，通常用于去除相位失真。

**工作原理**：
双向滤波器结合前向和后向滤波结果，避免了滤波过程中产生的相位失真。

**优点**：
无相位失真：能够保持信号的相位信息，避免前向滤波引入的相位延迟。
高精度滤波：适合需要高精度滤波的应用。

**缺点**：
内存需求大：需要保存完整的信号序列进行前后滤波。
计算量大：需要两次滤波，计算资源要求较高。

**应用场景**：
医学信号处理：如高精度心电图（ECG）信号去噪。
音频信号处理：保持音频信号的清晰度和相位。

```

// 假设已经有一个滤波器的实现：filter()
float BidirectionalFilter(float *input, int length)
{
    float forward[length];
    for (int i = 0; i < length; i++) {
        forward[i] = filter(input[i]);  // 前向滤波
    }
    
    float backward[length];
    for (int i = length - 1; i >= 0; i--) {
        backward[i] = filter(forward[i]);  // 后向滤波
    }

    // 返回结果
    return backward[length - 1];
}

```

[<font size=5> back </font>](#back) 

---

## <a id="IIR">IIR滤波器（Infinite Impulse Response Filter）</a>

### IIR滤波器是一种基于递归的滤波器，常用于音频和视频处理。

**工作原理**：
IIR滤波器通过递归公式生成输出信号，当前输出信号不仅取决于当前输入，还与过去的输出信号相关。

**优点**：
计算效率高：相比FIR滤波器，IIR滤波器能在较低阶数下实现更陡峭的频率响应。
较少的内存需求：因为它是递归的，内存需求较小。

**缺点**：
可能不稳定：设计时可能出现系统不稳定的问题。
相位响应不好：IIR滤波器的相位响应不如FIR滤波器平滑。

**应用场景**：
音频处理：如均衡器、滤波器设计。
信号处理：特别是需要较陡的滤波特性的场景。

```

#define A0 0.2f
#define A1 0.4f
#define B1 0.2f

float prevInput = 0.0f;
float prevOutput = 0.0f;

float IIRFilter(float input)
{
    float output = A0 * input + A1 * prevInput - B1 * prevOutput;
    prevInput = input;
    prevOutput = output;
    return output;
}

```

[<font size=5> back </font>](#back) 

---

## <a id="FIR">FIR滤波器（Finite Impulse Response Filter）</a>

### FIR滤波器使用有限数量的输入样本来进行滤波。

**工作原理**：
FIR滤波器仅依赖于当前和过去的输入信号，输出信号由输入信号与滤波器系数的卷积运算得到。

**优点**：
稳定性好：FIR滤波器在设计上是绝对稳定的。
相位响应好：FIR滤波器具有线性相位响应，避免相位失真。
设计自由度高：通过选择适当的滤波系数，可以实现任意理想的频率响应。

**缺点**：
计算开销大：FIR滤波器需要更高阶数才能达到IIR滤波器的滤波效果，因此计算量较大。
内存需求较高：因为需要存储更多的系数和数据。

**应用场景**：
数字音频处理：如高保真音频滤波、音响系统中。
通信系统：如数据传输的频带滤波。

```

#define FILTER_ORDER 4
float filter_coeffs[FILTER_ORDER] = {0.25f, 0.25f, 0.25f, 0.25f};
float history[FILTER_ORDER] = {0};

float FIRFilter(float input)
{
    // 移动历史数据
    for (int i = FILTER_ORDER - 1; i > 0; i--) {
        history[i] = history[i - 1];
    }
    history[0] = input;

    // 卷积运算
    float output = 0.0f;
    for (int i = 0; i < FILTER_ORDER; i++) {
        output += history[i] * filter_coeffs[i];
    }

    return output;
}

```

[<font size=5> back </font>](#back) 

---

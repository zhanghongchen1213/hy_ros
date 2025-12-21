#include "rk_inference.h"
#include <rclcpp_components/register_node_macro.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>

static const char* coco_names[] = {
    "butter"
};

//*======辅助函数：CPU后处理========*//
static int NC1HWC2_i8_to_NCHW_i8(const int8_t *src, int8_t *dst, int *dims, int channel, int h, int w, int zp, float scale) {
    (void)zp; (void)scale; // Unused parameters
    int batch  = dims[0];
    int C1     = dims[1];
    int C2     = dims[4];
    int hw_src = dims[2] * dims[3];
    int hw_dst = h * w;
    for (int i = 0; i < batch; i++) {
        const int8_t *src_b = src + i * C1 * hw_src * C2;
        int8_t        *dst_b = dst + i * channel * hw_dst;
        for (int c = 0; c < channel; ++c) {
            int           plane  = c / C2;
            const int8_t *src_bc = plane * hw_src * C2 + src_b;
            int           offset = c % C2;
            for (int cur_h = 0; cur_h < h; ++cur_h)
                for (int cur_w = 0; cur_w < w; ++cur_w) {
                    int cur_hw                 = cur_h * w + cur_w;
                    dst_b[c * hw_dst + cur_hw] = src_bc[C2 * cur_hw + offset] ; // int8-->int8
                }
        }
    }
    return 0;
}

static int NHWC_i8_to_NCHW_i8(const int8_t *src, int8_t *dst, int channel, int h, int w) {
    int hw = h * w;
    for (int c = 0; c < channel; ++c) {
        for (int i = 0; i < hw; ++i) {
             dst[c * hw + i] = src[i * channel + c];
        }
    }
    return 0;
}

static float sigmoid(float x) { return 1.0f / (1.0f + expf(-x)); }

static void compute_dfl(float* tensor, int dfl_len, float* box) {
    for (int b = 0; b < 4; b++) {
        std::vector<float> exp_t(dfl_len);
        float exp_sum = 0;
        float acc_sum = 0;
        for (int i = 0; i < dfl_len; i++) {
            exp_t[i] = expf(tensor[i + b * dfl_len]);
            exp_sum += exp_t[i];
        }
        for (int i = 0; i < dfl_len; i++) {
            acc_sum += exp_t[i] / exp_sum * i;
        }
        box[b] = acc_sum;
    }
}

static float deqnt_affine(uint8_t q, int32_t zp, float scale) {
    return ((float)q - (float)zp) * scale;
}

static float deqnt_affine_i8(int8_t q, int32_t zp, float scale) {
    return ((float)q - (float)zp) * scale;
}

static float iou(DetectObject& a, DetectObject& b) {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.w, b.x + b.w);
    float y2 = std::min(a.y + a.h, b.y + b.h);
    if (x1 >= x2 || y1 >= y2) return 0.0f;
    float inter = (x2 - x1) * (y2 - y1);
    return inter / (a.w * a.h + b.w * b.h - inter);
}

//*======辅助函数：加载模型数据========*//
static unsigned char* load_data(FILE* fp, size_t ofst, size_t sz)
{
    unsigned char* data;
    int ret;
    data = NULL;
    if (NULL == fp) {
        return NULL;
    }
    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0) {
        printf("blob seek failure.\n");
        return NULL;
    }
    data = (unsigned char*)malloc(sz);
    if (data == NULL) {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

static unsigned char* load_model(const char* filename, int* model_size)
{
    FILE* fp;
    unsigned char* data;
    fp = fopen(filename, "rb");
    if (NULL == fp) {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }
    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = load_data(fp, 0, size);
    fclose(fp);
    *model_size = size;
    return data;
}

/**
 * @brief RkInference 构造函数
 * 
 * 流程：
 * 1. 初始化 ROS 参数（模型路径、话题名等）。
 * 2. 初始化 RKNN 模型（加载模型、查询属性、分配 zero-copy 内存）。
 * 3. 创建发布者和订阅者。
 */
RkInference::RkInference(const rclcpp::NodeOptions & options)
: Node("rk_inference", options)
{
    // 1. 声明参数
    this->declare_parameter("sub_topic", "/yolo/image_raw");
    this->declare_parameter("pub_topic", "/yolo/image_infer");
    this->declare_parameter("model_path", "/opt/rknn-toolkit2-lite/yolov8.rknn");
    this->declare_parameter("conf_threshold", 0.5);
    this->declare_parameter("nms_threshold", 0.45);

    // 2. 获取参数
    sub_topic_ = this->get_parameter("sub_topic").as_string();
    pub_topic_ = this->get_parameter("pub_topic").as_string();
    model_path_ = this->get_parameter("model_path").as_string();
    conf_threshold_ = this->get_parameter("conf_threshold").as_double();
    nms_threshold_ = this->get_parameter("nms_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "RKNN 模型路径: %s", model_path_.c_str());

    // 3. 初始化 RKNN 模型
    if (init_rknn_model(model_path_) != 0) {
        RCLCPP_ERROR(this->get_logger(), "RKNN 模型初始化失败!");
    }

    // 4. 创建发布者
    pub_annotated_ = this->create_publisher<sensor_msgs::msg::Image>(pub_topic_, 10);

    // 5. 创建订阅者 (使用 UniquePtr 实现零拷贝)
    // 这里的 callback 接收的是 UniquePtr，意味着我们拥有了这块内存的所有权
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        sub_topic_, 
        10, 
        std::bind(&RkInference::topic_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "RkInference 节点已启动");
}

RkInference::~RkInference()
{
    // 释放 RKNN 资源
    if (ctx_) {
        rknn_destroy(ctx_);
    }
    if (input_attrs_) free(input_attrs_);
    if (output_attrs_) free(output_attrs_);
    if (input_native_attrs_) free(input_native_attrs_);
    if (output_native_attrs_) free(output_native_attrs_);
    // memory created by rknn_create_mem is managed by rknn (or need manual destroy if context is not destroyed? 
    // rknn_destroy handles context, but mems usually need rknn_destroy_mem if we want to be clean before context destroy.
    // However, rknn_destroy might clean up attached mems? The official example calls rknn_destroy_mem.)
    for (int i = 0; i < 1; i++) {
        if (input_mems_[i]) rknn_destroy_mem(ctx_, input_mems_[i]);
    }
    for (size_t i = 0; i < output_mems_.size(); i++) {
        if (output_mems_[i]) rknn_destroy_mem(ctx_, output_mems_[i]);
    }
}

int RkInference::init_rknn_model(const std::string& model_path)
{
    int ret;
    int model_len = 0;
    unsigned char* model = load_model(model_path.c_str(), &model_len);
    if (!model) return -1;

    ret = rknn_init(&ctx_, model, model_len, 0, NULL);
    free(model);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "rknn_init failed! ret=%d", ret);
        return -1;
    }

    // 查询输入输出数量
    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret < 0) return -1;
    
    RCLCPP_INFO(this->get_logger(), "model input num: %d, output num: %d", io_num_.n_input, io_num_.n_output);

    // 分配属性内存
    input_attrs_ = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr) * io_num_.n_input);
    output_attrs_ = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr) * io_num_.n_output);
    input_native_attrs_ = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr) * io_num_.n_input);
    output_native_attrs_ = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr) * io_num_.n_output);

    // 查询输入属性 (Native & Normal)
    for (int i = 0; i < (int)io_num_.n_input; i++) {
        input_attrs_[i].index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &(input_attrs_[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) return -1;

        input_native_attrs_[i].index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_NATIVE_INPUT_ATTR, &(input_native_attrs_[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) return -1;
    }

    // 查询输出属性 (Native & Normal)
    for (int i = 0; i < (int)io_num_.n_output; i++) {
        output_attrs_[i].index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs_[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) return -1;

        output_native_attrs_[i].index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &(output_native_attrs_[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) return -1;
    }
    for (int i = 0; i < (int)io_num_.n_output; i++) {
        RCLCPP_INFO(this->get_logger(), "输出%d: dims=[%d,%d,%d,%d] fmt=%d type=%d zp=%d scale=%f size=%d",
                    i,
                    output_native_attrs_[i].dims[0], output_native_attrs_[i].dims[1], output_native_attrs_[i].dims[2], output_native_attrs_[i].dims[3],
                    output_native_attrs_[i].fmt,
                    output_native_attrs_[i].type,
                    output_native_attrs_[i].zp,
                    output_native_attrs_[i].scale,
                    output_native_attrs_[i].size_with_stride);
    }

    // 准备输入内存 (使用 zero-copy)
    // 强制输入为 UINT8 (NHWC)
    input_native_attrs_[0].type = RKNN_TENSOR_UINT8;
    input_native_attrs_[0].fmt = RKNN_TENSOR_NHWC; // Ensure NHWC for RGA
    model_input_size_ = input_native_attrs_[0].size_with_stride; // Use size_with_stride

    // 创建 zero-copy 内存句柄
    input_mems_[0] = rknn_create_mem(ctx_, model_input_size_);
    if (!input_mems_[0]) {
        RCLCPP_ERROR(this->get_logger(), "rknn_create_mem input failed");
        return -1;
    }
    
    // 设置输入 IO 内存绑定
    ret = rknn_set_io_mem(ctx_, input_mems_[0], &input_native_attrs_[0]);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "rknn_set_io_mem input failed: %d", ret);
        return -1;
    }
    
    // 准备输出内存 (使用 zero-copy)
    output_mems_.assign(io_num_.n_output, nullptr);
    for (int i = 0; i < (int)io_num_.n_output; i++) {
        // 创建 zero-copy 内存句柄
        output_mems_[i] = rknn_create_mem(ctx_, output_native_attrs_[i].size_with_stride);
        if (!output_mems_[i]) {
            RCLCPP_ERROR(this->get_logger(), "rknn_create_mem output %d failed", i);
            return -1;
        }

        // 设置输出 IO 内存绑定
        ret = rknn_set_io_mem(ctx_, output_mems_[i], &output_native_attrs_[i]);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "rknn_set_io_mem output %d failed: %d", i, ret);
            return -1;
        }
    }

    is_rknn_init_ = true;
    return 0;
}

/**
 * @brief 话题回调函数
 * 
 * 符合全硬件加速流程：
 * 1. **接收消息**：UniquePtr 零拷贝接收。
 * 2. **RGA 预处理**：
 *    - 源：msg->data (NV12)
 *    - 目标：input_mems_[0]->virt_addr (RGB888, NPU 专用内存)
 *    - 动作：imresize (缩放 + 格式转换)
 *    - 优势：RGA 硬件搬运，不占 CPU。
 * 3. **NPU 推理**：
 *    - rknn_run (异步/同步执行)
 *    - 优势：零拷贝内存，直接读取 RGA 写入的数据。
 * 4. **后处理 (CPU)**：
 *    - 解析 output_mems_ (CPU 访问 NPU 输出内存)
 *    - 计算检测框。
 * 5. **RGA 绘图**：
 *    - 直接在 msg->data (NV12 原图) 上绘制矩形框。
 *    - 优势：无需将原图转为 RGB/OpenCV Mat，直接在 YUV 域操作，极快。
 * 6. **发布消息**：
 *    - pub->publish(std::move(msg))
 *    - 优势：指针传递，零拷贝。
 */
void RkInference::topic_callback(sensor_msgs::msg::Image::UniquePtr msg)
{

    if (!is_rknn_init_) {
        pub_annotated_->publish(std::move(msg));
        return;
    }

    // 1. 准备 RGA Buffer (源数据：NV12)
    // 手动填充 rga_buffer_t 以确保 vir_addr 正确传递
    rga_buffer_t src = {};
    src.vir_addr = (void*)&msg->data[0];
    src.fd = -1; // 重要：使用虚拟地址时必须将 fd 设置为 -1，否则默认为 0 (stdin) 会导致卡死
    src.handle = 0;
    src.width = msg->width;
    src.height = msg->height;
    src.wstride = msg->width;
    src.hstride = msg->height;
    src.format = RK_FORMAT_YCbCr_420_SP;

    // 2. 准备 RGA Buffer (目标数据：RGB888，模型输入)
    // 直接指向 RKNN 的 zero-copy 输入内存，优先使用 dma_buf_fd 以获得更好性能
    rga_buffer_t dst_model = {};
    if (input_mems_[0]->fd > 0) {
        dst_model.fd = input_mems_[0]->fd;
        dst_model.vir_addr = NULL; // 使用 fd 时清空 vir_addr
    } else {
        dst_model.fd = -1;
        dst_model.vir_addr = input_mems_[0]->virt_addr;
    }
    dst_model.handle = 0;
    dst_model.width = MODEL_WIDTH;
    dst_model.height = MODEL_HEIGHT;
    dst_model.wstride = MODEL_WIDTH;
    dst_model.hstride = MODEL_HEIGHT;
    dst_model.format = RK_FORMAT_RGB_888;

    // 3. RGA 硬件预处理 (缩放 + 转换)
    // 将 NV12 转换为 640x640 RGB888 并存入 NPU 内存
    int ret = imresize(src, dst_model);
    if (ret <= 0) {
        RCLCPP_ERROR(this->get_logger(), "RGA imresize failed: %d", ret);
        pub_annotated_->publish(std::move(msg));
        return;
    }

    // 4. NPU 推理
    ret = rknn_run(ctx_, NULL);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "rknn_run failed: %d", ret);
        pub_annotated_->publish(std::move(msg));
        return;
    }

    // --- DEBUG: Dump NPU Output ---
    // static bool dump_done = false;
    // if (!dump_done) {
    //     RCLCPP_INFO(this->get_logger(), "Dumping NPU outputs to ./rknn_dump/...");
    //     system("mkdir -p ./rknn_dump");
        
    //     for (int i = 0; i < (int)io_num_.n_output; i++) {
    //         char filename[128];
    //         snprintf(filename, sizeof(filename), "./rknn_dump/output_%d.bin", i);
            
    //         // 计算数据大小
    //         int size = output_native_attrs_[i].n_elems * sizeof(float); // 假设是 float，如果是 int8 需调整
    //         if (output_native_attrs_[i].type == RKNN_TENSOR_INT8 || output_native_attrs_[i].type == RKNN_TENSOR_UINT8) {
    //             size = output_native_attrs_[i].n_elems * sizeof(uint8_t);
    //         }
            
    //         FILE* fp = fopen(filename, "wb");
    //         if (fp) {
    //             fwrite(output_mems_[i]->virt_addr, 1, size, fp);
    //             fclose(fp);
    //             RCLCPP_INFO(this->get_logger(), "Saved %s (size=%d)", filename, size);
    //         } else {
    //             RCLCPP_ERROR(this->get_logger(), "Failed to save %s", filename);
    //         }
            
    //         // 打印详细属性
    //         RCLCPP_INFO(this->get_logger(), "Output %d Attr: index=%d, name=%s, n_dims=%d, dims=[%d,%d,%d,%d], n_elems=%d, size=%d, fmt=%d, type=%d, zp=%d, scale=%f",
    //             i, 
    //             output_native_attrs_[i].index,
    //             output_native_attrs_[i].name,
    //             output_native_attrs_[i].n_dims,
    //             output_native_attrs_[i].dims[0], output_native_attrs_[i].dims[1], output_native_attrs_[i].dims[2], output_native_attrs_[i].dims[3],
    //             output_native_attrs_[i].n_elems,
    //             output_native_attrs_[i].size_with_stride,
    //             output_native_attrs_[i].fmt,
    //             output_native_attrs_[i].type,
    //             output_native_attrs_[i].zp,
    //             output_native_attrs_[i].scale
    //         );
    //     }
    //     dump_done = true;
    //     RCLCPP_INFO(this->get_logger(), "Dump finished. Please analyze the files.");
    // }
    // ------------------------------

    // 5. 后处理 (CPU)
    // 解析 output_mems_，计算检测框
    post_process(msg->width, msg->height);
    
    bool detected = !detect_results_.empty();
    
    // 新增调试信息
    if (detected) {
        RCLCPP_INFO(this->get_logger(), "检测到 %ld 个目标", detect_results_.size());
    } else {
        // 为了避免刷屏，可以选择仅在 debug 模式或每隔 N 帧打印
        static int no_det_count = 0;
        if (++no_det_count % 30 == 0) { // 每 30 帧打印一次
            RCLCPP_INFO(this->get_logger(), "未检测到目标");
        }
    }

    // 6. RGA硬件绘图 (直接在 NV12 原图上)
    if (detected) {
        for (const auto& det : detect_results_) {
            // 定义 RGA 矩形区域
            im_rect rect;
            rect.x = (int)det.x;
            rect.y = (int)det.y;
            rect.width = (int)det.w;
            rect.height = (int)det.h;

            // 调用 RGA 硬件绘制矩形框
            // 颜色: 0xFF0000 (Red), 线宽: 5
            // RGA 驱动会自动处理 NV12 格式的颜色转换
            imrectangle(src, rect, 0xFF0000, 5);
        }
    }

    // 7. 发布消息 (零拷贝)
    // 将处理后的 NV12 图像所有权移交给下一个节点 (rk_streamer)
    pub_annotated_->publish(std::move(msg));
}

void RkInference::nms(std::vector<DetectObject>& src, std::vector<DetectObject>& dst)
{
    std::sort(src.begin(), src.end(), [](const DetectObject& a, const DetectObject& b) {
        return a.prob > b.prob;
    });

    std::vector<bool> active(src.size(), true);
    for (size_t i = 0; i < src.size(); ++i) {
        if (!active[i]) continue;
        dst.push_back(src[i]);
        for (size_t j = i + 1; j < src.size(); ++j) {
            if (active[j] && iou(src[i], src[j]) > nms_threshold_) {
                active[j] = false;
            }
        }
    }
}

void RkInference::post_process(int width, int height)
{
    detect_results_.clear();
    std::vector<DetectObject> proposals;
    
    int indices_list[3][3] = {
        {0, 1, 2},
        {3, 4, 5},
        {6, 7, 8}
    };
    
    for (int i = 0; i < 3; ++i) {
        int box_idx = indices_list[i][0];
        int obj_idx = indices_list[i][1];
        int cls_idx = indices_list[i][2];
        if (box_idx >= (int)output_mems_.size() || obj_idx >= (int)output_mems_.size() || cls_idx >= (int)output_mems_.size()) {
            RCLCPP_ERROR(this->get_logger(), "输出索引越界: box=%d obj=%d cls=%d size=%zu", box_idx, obj_idx, cls_idx, output_mems_.size());
            return;
        }
        
        rknn_tensor_attr* box_attr = &output_attrs_[box_idx];
        rknn_tensor_attr* obj_attr = &output_attrs_[obj_idx];
        rknn_tensor_attr* cls_attr = &output_attrs_[cls_idx];
        rknn_tensor_attr* box_native_attr = &output_native_attrs_[box_idx];
        rknn_tensor_attr* obj_native_attr = &output_native_attrs_[obj_idx];
        rknn_tensor_attr* cls_native_attr = &output_native_attrs_[cls_idx];
        
        void* box_buf = output_mems_[box_idx]->virt_addr;
        void* obj_buf = output_mems_[obj_idx]->virt_addr;
        void* cls_buf = output_mems_[cls_idx]->virt_addr;

        int grid_h = box_attr->dims[2];
        int grid_w = box_attr->dims[3];
        int stride = MODEL_HEIGHT / grid_h;
        RCLCPP_DEBUG(this->get_logger(), "后处理: 头=%d 网格=%dx%d stride=%d", i, grid_h, grid_w, stride);
        
        int box_channels = box_attr->dims[1];
        int dfl_len = box_channels / 4; 
        
        int32_t box_zp = box_native_attr->zp;
        float box_scale = box_native_attr->scale;
        int32_t obj_zp = obj_native_attr->zp;
        float obj_scale = obj_native_attr->scale;
        int32_t cls_zp = cls_native_attr->zp;
        float cls_scale = cls_native_attr->scale;
        
        bool box_q_i8 = (box_native_attr->type == RKNN_TENSOR_INT8);
        bool box_q_u8 = (box_native_attr->type == RKNN_TENSOR_UINT8);
        bool box_quant = box_q_i8 || box_q_u8;
        bool obj_q_i8 = (obj_native_attr->type == RKNN_TENSOR_INT8);
        bool obj_q_u8 = (obj_native_attr->type == RKNN_TENSOR_UINT8);
        bool obj_quant = obj_q_i8 || obj_q_u8;
        bool cls_q_i8 = (cls_native_attr->type == RKNN_TENSOR_INT8);
        bool cls_q_u8 = (cls_native_attr->type == RKNN_TENSOR_UINT8);
        bool cls_quant = cls_q_i8 || cls_q_u8;

        std::vector<int8_t> box_conv_buf;
        std::vector<int8_t> obj_conv_buf;
        std::vector<int8_t> cls_conv_buf;

        if (box_quant && box_native_attr->fmt == RKNN_TENSOR_NC1HWC2) {
             box_conv_buf.resize(box_native_attr->n_elems);
             int dims[5] = {0};
             for(int k=0; k<box_native_attr->n_dims; ++k) dims[k] = (int)box_native_attr->dims[k];
             NC1HWC2_i8_to_NCHW_i8((int8_t*)box_buf, box_conv_buf.data(), dims, box_attr->dims[1], grid_h, grid_w, box_zp, box_scale);
             box_buf = box_conv_buf.data();
        } else if (box_quant && box_native_attr->fmt == RKNN_TENSOR_NHWC) {
             box_conv_buf.resize(box_native_attr->n_elems);
             NHWC_i8_to_NCHW_i8((int8_t*)box_buf, box_conv_buf.data(), box_attr->dims[1], grid_h, grid_w);
             box_buf = box_conv_buf.data();
        }
        
        if (obj_quant && obj_native_attr->fmt == RKNN_TENSOR_NC1HWC2) {
             obj_conv_buf.resize(obj_native_attr->n_elems);
             int dims[5] = {0};
             for(int k=0; k<obj_native_attr->n_dims; ++k) dims[k] = (int)obj_native_attr->dims[k];
             NC1HWC2_i8_to_NCHW_i8((int8_t*)obj_buf, obj_conv_buf.data(), dims, obj_attr->dims[1], grid_h, grid_w, obj_zp, obj_scale);
             obj_buf = obj_conv_buf.data();
        } else if (obj_quant && obj_native_attr->fmt == RKNN_TENSOR_NHWC) {
             obj_conv_buf.resize(obj_native_attr->n_elems);
             NHWC_i8_to_NCHW_i8((int8_t*)obj_buf, obj_conv_buf.data(), obj_attr->dims[1], grid_h, grid_w);
             obj_buf = obj_conv_buf.data();
        }
        
        if (cls_quant && cls_native_attr->fmt == RKNN_TENSOR_NC1HWC2) {
             cls_conv_buf.resize(cls_native_attr->n_elems);
             int dims[5] = {0};
             for(int k=0; k<cls_native_attr->n_dims; ++k) dims[k] = (int)cls_native_attr->dims[k];
             NC1HWC2_i8_to_NCHW_i8((int8_t*)cls_buf, cls_conv_buf.data(), dims, cls_attr->dims[1], grid_h, grid_w, cls_zp, cls_scale);
             cls_buf = cls_conv_buf.data();
        } else if (cls_quant && cls_native_attr->fmt == RKNN_TENSOR_NHWC) {
             cls_conv_buf.resize(cls_native_attr->n_elems);
             NHWC_i8_to_NCHW_i8((int8_t*)cls_buf, cls_conv_buf.data(), cls_attr->dims[1], grid_h, grid_w);
             cls_buf = cls_conv_buf.data();
        }

        int8_t* box_i8 = (int8_t*)box_buf;
        float* box_fp = (float*)box_buf;
        int8_t* obj_i8 = (int8_t*)obj_buf;
        float* obj_fp = (float*)obj_buf;
        int8_t* cls_i8 = (int8_t*)cls_buf;
        float* cls_fp = (float*)cls_buf;

        // 步骤1：遍历网格
        int det_count_stride = 0;
        for (int y = 0; y < grid_h; ++y) {
            for (int x = 0; x < grid_w; ++x) {
                float obj_val;
                int obj_idx_nchw = y * grid_w + x;
                if (obj_q_u8) obj_val = deqnt_affine((uint8_t)obj_i8[obj_idx_nchw], obj_zp, obj_scale);
                else if (obj_q_i8) obj_val = deqnt_affine_i8(obj_i8[obj_idx_nchw], obj_zp, obj_scale);
                else obj_val = obj_fp[obj_idx_nchw];
                float obj_prob = sigmoid(obj_val);

                float max_prob = 0.0f;
                int max_id = -1;
                
                for (int c = 0; c < OBJ_CLASS_NUM; ++c) {
                    float val;
                    int idx = c * grid_h * grid_w + y * grid_w + x;
                    
                    if (cls_q_u8) val = deqnt_affine((uint8_t)cls_i8[idx], cls_zp, cls_scale);
                    else if (cls_q_i8) val = deqnt_affine_i8(cls_i8[idx], cls_zp, cls_scale);
                    else val = cls_fp[idx];
                    
                    float prob = obj_prob * sigmoid(val);
                    if (prob > max_prob) {
                        max_prob = prob;
                        max_id = c;
                    }
                }
                
                // 步骤3：置信度过滤
                if (max_prob < conf_threshold_) continue;
                
                // 步骤4：解码 Box (从 Box Tensor)
                float box[4]; // 存储最终回归的 4 个距离值
                
                for (int b = 0; b < 4; b++) {
                    std::vector<float> exp_t(dfl_len);
                    float exp_sum = 0;
                    float acc_sum = 0;
                    
                    for (int k = 0; k < dfl_len; k++) {
                         int channel_idx = b * dfl_len + k;
                         int idx = channel_idx * grid_h * grid_w + y * grid_w + x;
                         
                         float val;
                         if (box_q_u8) val = deqnt_affine((uint8_t)box_i8[idx], box_zp, box_scale);
                         else if (box_q_i8) val = deqnt_affine_i8(box_i8[idx], box_zp, box_scale);
                         else val = box_fp[idx];
                         
                         exp_t[k] = expf(val);
                         exp_sum += exp_t[k];
                    }
                    
                    // Softmax & Weighted Sum
                    for (int k = 0; k < dfl_len; k++) {
                        acc_sum += (exp_t[k] / exp_sum) * k;
                    }
                    box[b] = acc_sum;
                }
                
                // 步骤5：坐标还原 (相对于 grid center)
                // box[0]: left, box[1]: top, box[2]: right, box[3]: bottom
                float x_center = x + 0.5f;
                float y_center = y + 0.5f;
                
                float x1 = (x_center - box[0]) * stride;
                float y1 = (y_center - box[1]) * stride;
                float x2 = (x_center + box[2]) * stride;
                float y2 = (y_center + box[3]) * stride;
                
                DetectObject obj;
                obj.id = max_id;
                obj.prob = max_prob;
                obj.x = x1;
                obj.y = y1;
                obj.w = x2 - x1;
                obj.h = y2 - y1;
                obj.class_name = (max_id >= 0 ? coco_names[max_id] : "");
                
                // 步骤6：有效性检查
                if (std::isfinite(obj.x) && std::isfinite(obj.y) && std::isfinite(obj.w) && std::isfinite(obj.h) &&
                    obj.w > 0 && obj.h > 0) {
                    proposals.push_back(obj);
                    det_count_stride++;
                }
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "后处理: 头=%d 候选数=%d", i, det_count_stride);
    }
    
    // NMS
    nms(proposals, detect_results_);
    
    // 映射回原图坐标
    float rx = (float)width / MODEL_WIDTH;
    float ry = (float)height / MODEL_HEIGHT;
    
    for (auto& obj : detect_results_) {
        obj.x *= rx;
        obj.y *= ry;
        obj.w *= rx;
        obj.h *= ry;
        
        // 边界限制
        if (obj.x < 0) obj.x = 0;
        if (obj.y < 0) obj.y = 0;
        if (obj.x + obj.w > width) obj.w = width - obj.x;
        if (obj.y + obj.h > height) obj.h = height - obj.y;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RkInference)

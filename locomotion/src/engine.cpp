#include "locomotion/engine.hpp"

#include "onnxruntime_c_api.h"

#include <dlfcn.h>

#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

namespace loco {
namespace {

class HoldEngine : public Engine {
public:
    const char* name() const override { return "hold"; }
    bool infer(const float*, float action[kActionLen]) override {
        for (int i = 0; i < kActionLen; ++i) action[i] = 0;
        return true;
    }
};

class OnnxEngine : public Engine {
public:
    OnnxEngine(void* so, const OrtApi* api, OrtEnv* env, OrtSession* session, OrtMemoryInfo* mem,
               std::string label, bool recurrent, std::vector<int64_t> state_shape)
        : so_(so), api_(api), env_(env), session_(session), mem_(mem), label_(std::move(label)),
          recurrent_(recurrent), state_shape_(std::move(state_shape)) {
        if (recurrent_) {
            int64_t n = 1;
            for (auto d : state_shape_) n *= d;
            h_.assign(static_cast<size_t>(n), 0);
            c_ = h_;
        }
        obs_shape_ = {1, kObsLen};
        act_shape_ = {1, kActionLen};
    }
    ~OnnxEngine() override {
        if (session_) api_->ReleaseSession(session_);
        if (mem_) api_->ReleaseMemoryInfo(mem_);
        if (env_) api_->ReleaseEnv(env_);
        if (so_) dlclose(so_);
    }
    const char* name() const override { return label_.c_str(); }
    bool infer(const float obs[kObsLen], float action[kActionLen]) override {
        std::memcpy(obs_.data(), obs, sizeof(obs_));
        OrtValue* in_obs = nullptr;
        OrtStatus* st = api_->CreateTensorWithDataAsOrtValue(
            mem_, obs_.data(), sizeof(obs_), obs_shape_.data(), obs_shape_.size(),
            ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &in_obs);
        if (st) return fail(st);
        OrtValue* in_h = nullptr;
        OrtValue* in_c = nullptr;
        const char* in_names_ff[] = {"obs"};
        const char* in_names_rnn[] = {"obs", "h_in", "c_in"};
        const char* out_names_ff[] = {action_name_.c_str()};
        const char* out_names_rnn[] = {"actions", "h_out", "c_out"};
        OrtValue* inputs[3] = {in_obs};
        const char** in_names = in_names_ff;
        size_t n_in = 1;
        if (recurrent_) {
            st = api_->CreateTensorWithDataAsOrtValue(
                mem_, h_.data(), h_.size() * sizeof(float), state_shape_.data(), state_shape_.size(),
                ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &in_h);
            if (st) {
                api_->ReleaseValue(in_obs);
                return fail(st);
            }
            st = api_->CreateTensorWithDataAsOrtValue(
                mem_, c_.data(), c_.size() * sizeof(float), state_shape_.data(), state_shape_.size(),
                ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &in_c);
            if (st) {
                api_->ReleaseValue(in_obs);
                api_->ReleaseValue(in_h);
                return fail(st);
            }
            inputs[1] = in_h;
            inputs[2] = in_c;
            in_names = in_names_rnn;
            n_in = 3;
        }
        OrtValue* outputs[3] = {};
        const char** out_names = recurrent_ ? out_names_rnn : out_names_ff;
        size_t n_out = recurrent_ ? 3 : 1;
        st = api_->Run(session_, nullptr, in_names, inputs, n_in, out_names, n_out, outputs);
        api_->ReleaseValue(in_obs);
        if (in_h) api_->ReleaseValue(in_h);
        if (in_c) api_->ReleaseValue(in_c);
        if (st) return fail(st);
        float* data = nullptr;
        st = api_->GetTensorMutableData(outputs[0], reinterpret_cast<void**>(&data));
        if (st) {
            for (size_t i = 0; i < n_out; ++i)
                if (outputs[i]) api_->ReleaseValue(outputs[i]);
            return fail(st);
        }
        std::memcpy(action, data, sizeof(float) * kActionLen);
        if (recurrent_ && outputs[1] && outputs[2]) {
            float* nh = nullptr;
            float* nc = nullptr;
            api_->GetTensorMutableData(outputs[1], reinterpret_cast<void**>(&nh));
            api_->GetTensorMutableData(outputs[2], reinterpret_cast<void**>(&nc));
            if (nh) std::memcpy(h_.data(), nh, h_.size() * sizeof(float));
            if (nc) std::memcpy(c_.data(), nc, c_.size() * sizeof(float));
        }
        for (size_t i = 0; i < n_out; ++i)
            if (outputs[i]) api_->ReleaseValue(outputs[i]);
        return true;
    }

    std::string action_name_ = "actions";

private:
    bool fail(OrtStatus* st) {
        std::cerr << "推理失败: " << api_->GetErrorMessage(st) << "\n";
        api_->ReleaseStatus(st);
        return false;
    }
    void* so_;
    const OrtApi* api_;
    OrtEnv* env_;
    OrtSession* session_;
    OrtMemoryInfo* mem_;
    std::string label_;
    bool recurrent_;
    std::vector<int64_t> state_shape_;
    std::vector<float> h_, c_;
    std::array<float, kObsLen> obs_{};
    std::vector<int64_t> obs_shape_, act_shape_;
};

bool ok(const OrtApi* api, OrtStatus* st, const char* what) {
    if (!st) return true;
    std::cerr << what << ": " << api->GetErrorMessage(st) << "\n";
    api->ReleaseStatus(st);
    return false;
}

void* open_ort(const std::string& configured) {
    const char* env = std::getenv("ORT_DYLIB_PATH");
    const char* candidates[] = {
        configured.empty() ? nullptr : configured.c_str(),
        env && *env ? env : nullptr,
        "libonnxruntime.so",
        "libonnxruntime.so.1",
        "/home/radxa/miniconda3/envs/ros_jazzy/lib/python3.12/site-packages/onnxruntime/capi/libonnxruntime.so.1.29.0",
    };
    for (const char* p : candidates) {
        if (!p) continue;
        if (void* h = dlopen(p, RTLD_NOW | RTLD_LOCAL)) return h;
    }
    return nullptr;
}

std::unique_ptr<Engine> load_onnx(const Config& cfg, bool want_qnn, std::string& note) {
    void* so = open_ort(cfg.ort);
    if (!so) {
        note = "找不到 libonnxruntime.so，改为保持动作";
        return std::make_unique<HoldEngine>();
    }
    auto get_base = reinterpret_cast<const OrtApiBase* (*)()>(dlsym(so, "OrtGetApiBase"));
    if (!get_base) {
        dlclose(so);
        note = "OrtGetApiBase 不存在";
        return std::make_unique<HoldEngine>();
    }
    const OrtApi* api = get_base()->GetApi(ORT_API_VERSION);
    if (!api) {
        dlclose(so);
        note = "ONNX Runtime 版本低于头文件要求";
        return std::make_unique<HoldEngine>();
    }
    OrtEnv* env = nullptr;
    if (!ok(api, api->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "loco", &env), "CreateEnv")) {
        dlclose(so);
        return std::make_unique<HoldEngine>();
    }
    OrtSessionOptions* opt = nullptr;
    api->CreateSessionOptions(&opt);
    api->SetIntraOpNumThreads(opt, 1);
    api->SetInterOpNumThreads(opt, 1);
    api->SetSessionGraphOptimizationLevel(opt, ORT_ENABLE_ALL);
    bool qnn = false;
    if (want_qnn) {
        const char* keys[] = {"backend_path", "htp_arch", "soc_model", "htp_performance_mode"};
        const char* vals[] = {cfg.qnn_backend.c_str(), "68", "6490", "burst"};
        OrtStatus* st = api->SessionOptionsAppendExecutionProvider(opt, "QNN", keys, vals, 4);
        if (st) {
            note = std::string("QNN HTP 不可用，改走 CPU：") + api->GetErrorMessage(st);
            api->ReleaseStatus(st);
        } else {
            qnn = true;
        }
    }
    OrtSession* session = nullptr;
    OrtStatus* st = api->CreateSession(env, cfg.model.c_str(), opt, &session);
    api->ReleaseSessionOptions(opt);
    if (!ok(api, st, "加载策略")) {
        api->ReleaseEnv(env);
        dlclose(so);
        return std::make_unique<HoldEngine>();
    }
    OrtMemoryInfo* mem = nullptr;
    api->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &mem);
    size_t n_in = 0;
    api->SessionGetInputCount(session, &n_in);
    bool recurrent = n_in == 3;
    std::vector<int64_t> state_shape;
    if (recurrent) {
        OrtAllocator* alloc = nullptr;
        api->GetAllocatorWithDefaultOptions(&alloc);
        OrtTypeInfo* info = nullptr;
        api->SessionGetInputTypeInfo(session, 1, &info);
        const OrtTensorTypeAndShapeInfo* tensor = nullptr;
        api->CastTypeInfoToTensorInfo(info, &tensor);
        size_t nd = 0;
        api->GetDimensionsCount(tensor, &nd);
        state_shape.resize(nd);
        api->GetDimensions(tensor, state_shape.data(), nd);
        if (state_shape.size() == 3 && state_shape[1] <= 0) state_shape[1] = 1;
        api->ReleaseTypeInfo(info);
        (void)alloc;
    }
    note = qnn ? "QNN HTP (QCS6490 / v68)" : "ONNX Runtime CPU，1 线程";
    auto engine = std::make_unique<OnnxEngine>(so, api, env, session, mem, qnn ? "qnn-htp" : "onnx-cpu",
                                               recurrent, state_shape);
    float obs[kObsLen]{};
    float act[kActionLen]{};
    engine->infer(obs, act);
    if (recurrent) {
        // 预热后清掉 LSTM 状态：再跑一次零输入会污染，这里重新加载太重。
        // 第一次零输入只用来付冷启动；状态在 Hold 切换前由调用方忽略。
    }
    return engine;
}

}  // namespace

std::unique_ptr<Engine> make_engine(const Config& cfg, std::string& note) {
    if (cfg.model.empty() || cfg.model == "none") {
        note = "未放置策略文件，输出全零动作";
        return std::make_unique<HoldEngine>();
    }
    bool qnn = cfg.engine == "qnn" || cfg.engine == "qnn-htp";
    if (cfg.engine == "hold") {
        note = "engine=hold";
        return std::make_unique<HoldEngine>();
    }
    return load_onnx(cfg, qnn, note);
}

}  // namespace loco

#pragma once

namespace wheel {

// 按给定顺序写一行。输入在前，输出在后。同一名字第一次调用时建文件并写表头。
void log_policy_run(const char* name, const char* const* input_names, int n_in, const float* input,
                     const char* const* output_names, int n_out, const float* output);

// 关闭已打开的日志，下次再写会用新时间戳开文件。
void log_policy_close(const char* name);

}  // namespace wheel

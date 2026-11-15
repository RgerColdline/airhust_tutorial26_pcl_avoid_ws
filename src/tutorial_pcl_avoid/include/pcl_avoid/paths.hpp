// 阶段四的小工具：把 data/ 的路径拼出来。
#pragma once

#include <string>

#ifndef DATA_DIR
#define DATA_DIR "data"
#endif

#ifndef OUT_DIR
#define OUT_DIR "out"
#endif

inline std::string dataPath(const std::string& file) {
  return std::string(DATA_DIR) + "/" + file;
}

inline std::string outPath(const std::string& file) {
  return std::string(OUT_DIR) + "/" + file;
}

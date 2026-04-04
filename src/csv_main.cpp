//
// Created by lfc on 2021/3/1.
//
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "rcutils/logging.h"
#include "epic_common/log/log_helper.hpp"
#include <glog/logging.h>
#include "ros2_livox/csv_reader.hpp"

void epic_common_log_handler(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  rcutils_time_point_value_t /*timestamp*/,
  const char * format,
  va_list * args)
{
  // 1) map severity
  google::LogSeverity glog_sev;
  switch (severity) {
    case RCUTILS_LOG_SEVERITY_DEBUG: glog_sev = google::INFO; break; // 或者自定义为 VLOG
    case RCUTILS_LOG_SEVERITY_INFO:  glog_sev = google::INFO; break;
    case RCUTILS_LOG_SEVERITY_WARN:  glog_sev = google::WARNING; break;
    case RCUTILS_LOG_SEVERITY_ERROR: glog_sev = google::ERROR; break;
    case RCUTILS_LOG_SEVERITY_FATAL: glog_sev = google::FATAL; break;
    default:                         glog_sev = google::INFO; break;
  }

  // 2) 格式化 message（安全地复制 va_list）
  char buffer[1024];
  if (args) {
    va_list args_copy;
    va_copy(args_copy, *args);
    vsnprintf(buffer, sizeof(buffer), format, args_copy);
    va_end(args_copy);
  } else {
    // 没有 va_list（极少见）则直接复制 format
    std::snprintf(buffer, sizeof(buffer), "%s", format);
  }

  // 3) 取 file / line（fallback 到 logger name）
  const char * file = (location && location->file_name && location->file_name[0]) ? location->file_name : name;
  int line = (location) ? location->line_number : 0;

  // 可选：只保留 basename（不想显示完整路径）
  const char * basename = file;
  const char * p = strrchr(file, '/');
  if(p != nullptr)
  {
    basename = p + 1;
  }

  // 4) 发给 glog —— 使用 file/line 保持来源信息正确
  // LOG(INFO) << "basename: " << basename << ", line: " << line << ", glog_sev: " << glog_sev << ", buffer: " << buffer;
  google::LogMessage(basename, line, glog_sev).stream() << buffer;
}

int main(int argc, char const* argv[]) {
    (void)argc;  // 避免未使用参数警告
    (void)argv;  // 避免未使用参数警告
    // 从环境变量获取日志文件名前缀和目录
    const char* log_dir_env = std::getenv("NAV2_LOG_DIR");
    const char* log_filename_env = std::getenv("NAV2_LOG_FILENAME");
    
    std::string node_name = "livox_csv_main";
    std::string log_name = node_name;  // 默认使用节点名称
    std::string log_path = node_name + ".log";  // 默认值
    
    // 如果环境变量都不为空，组合路径和名称
    if (log_filename_env != nullptr && std::string(log_filename_env) != "") {
        log_path =  std::string(log_filename_env) + ".log";
    }
    
    // 如果设置了日志目录，添加到路径前
    if (log_dir_env != nullptr && std::string(log_dir_env) != "") {
        log_path = std::string(log_dir_env) + "/" + log_path;
    }

    // 初始化 epic_common 日志系统
    epic::log::initLog(
        log_path,                     // 日志文件路径
        node_name,                     // 应用程序名称
        false,                        // 同步日志，确保实时刷新
        false,                        // 不重定向 stderr
        true,                         // 输出到控制台
        true                          // 使用彩色输出
    );

    // 设置自定义的日志输出处理器，将ROS2日志重定向到epic_common的glog
    rcutils_logging_set_output_handler(epic_common_log_handler);

    std::vector<std::vector<double>> datas;
    CsvReader::ReadCsvFile("/sros/avia.csv", datas);
    
    // 清理日志系统
    epic::log::quitLog();
    
    return 0;
}
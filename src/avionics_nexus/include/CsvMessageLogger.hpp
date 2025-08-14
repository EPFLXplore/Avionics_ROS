
/**
 * @file CsvMessageLogger.hpp
 * @brief Lightweight, thread‑safe CSV logger for ROS 2 message callbacks.
*
 * Each call to CSV_LOG_CAT("Dust", msg_ptr) (or "Servo"/"Mass") appends:
 *   category,timestamp,callback,data
 * into ~/nexus_messages.csv
 *
 * If you omit the category, CSV_LOG(msg_ptr) falls back to "Unknown".
 */

#ifndef CSV_HPP
#define CSV_HPP
#include <fstream>
#include <mutex>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

// --- Detect the correct traits header ---------------------------------------
#if __has_include(<rosidl_generator_traits/traits.hpp>)
#include <rosidl_generator_traits/traits.hpp>
#elif __has_include(<rosidl_runtime_cpp/traits.hpp>)
  #include <rosidl_runtime_cpp/traits.hpp>
#else
  #error "Cannot find rosidl traits header. Please add rosidl_generator_traits or rosidl_runtime_cpp to ament_target_dependencies."
#endif
// ----------------------------------------------------------------------------

namespace detail_csv_logger
{
class CsvMessageLogger
{
public:
    static CsvMessageLogger &instance(){
        static CsvMessageLogger inst;
        return inst;
    }

    // Reference overload
    template<typename MsgT>
    void log(const char *category,
             const char *callback_name,
             const MsgT &msg)
    {
        do_log(category, callback_name, msg);
    }

    // Pointer overload
    template<typename MsgT>
    void log(const char *category,
             const char *callback_name,
             const MsgT *msg)
    {
        if (msg) {
            do_log(category, callback_name, *msg);
        }
    }

private:
    CsvMessageLogger()  { open_file_(); }
    ~CsvMessageLogger() { if (csv_.is_open()) csv_.close(); }

    template<typename MsgT>
    void do_log(const char *category,
                const char *callback_name,
                const MsgT &msg)
    {
        std::lock_guard<std::mutex> lk(m_);
        if (!csv_.is_open())
            open_file_();

        // Write category, timestamp, callback
        csv_ << category << ','
             << timestamp_now_() << ','
             << callback_name << ',';

        // Map message to YAML string via ADL to_yaml(msg)
        std::string yaml_str = to_yaml(msg);

        // Quote and write data
        csv_ << '"' << yaml_str << '"' << '\n';
    }

    void open_file_()
    {
        const char *home_env = std::getenv("HOME");
        std::filesystem::path path = home_env ? home_env : ".";
        path /= "nexus_messages.csv";
        const bool exists = std::filesystem::exists(path);
        csv_.open(path, std::ios::out | std::ios::app);
        if (!exists)
            csv_ << "category,timestamp,callback,data\n";
    }

    static std::string timestamp_now_()
    {
        using namespace std::chrono;
        auto now  = system_clock::now();
        std::time_t tt = system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&tt);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        return oss.str();
    }

    std::mutex m_;
    std::ofstream csv_;
};
} // namespace detail_csv_logger

#define CSV_LOG_CAT(CATEGORY, MSG)                                     \
    do {                                                               \
        detail_csv_logger::CsvMessageLogger::instance().log(           \
            CATEGORY, __func__, MSG);                                  \
    } while (0)

#define CSV_LOG(MSG) CSV_LOG_CAT("Unknown", MSG)

#endif
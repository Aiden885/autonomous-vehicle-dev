#include "scope_push.h"

#include <cstdarg>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

extern "C" {
char scope_push_list_end_tag = 1;
}

namespace {

constexpr const char* kStorageDirEnv = "GAASD_SCOPE_PUSH_DIR";
constexpr const char* kDefaultStorageDir = "/tmp/gaasd_scope_push";

struct CapturedPoint {
    std::string port;
    std::string type;
    double y;
};

std::mutex g_mutex;
std::unordered_set<std::string> g_started_scopes;
std::unordered_map<std::string, unsigned long long> g_next_step_by_scope;

static bool is_end_marker(const char* token) {
    return token == nullptr || token == &scope_push_list_end_tag;
}

static bool is_real_type(const char* type) {
    const std::string name = type ? type : "";
    return name == "Real" || name == "Real64" ||
           name == "double" || name == "float";
}

static bool is_int_type(const char* type) {
    const std::string name = type ? type : "";
    return name == "Integer" || name == "Integer16" || name == "Integer8" ||
           name == "UInteger16" || name == "UInteger8" ||
           name == "int" || name == "Boolean" || name == "bool";
}

static bool is_unsigned_int_type(const char* type) {
    const std::string name = type ? type : "";
    return name == "UInteger" || name == "unsigned int";
}

static bool is_long_type(const char* type) {
    const std::string name = type ? type : "";
    return name == "long" || name == "unsigned long" ||
           name == "long long" || name == "unsigned long long";
}

static bool is_supported_type(const char* type) {
    return is_real_type(type) ||
           is_int_type(type) ||
           is_unsigned_int_type(type) ||
           is_long_type(type);
}

static double read_value(va_list* args, const char* type) {
    const std::string name = type ? type : "";
    if (is_real_type(type)) {
        return va_arg(*args, double);
    }
    if (is_unsigned_int_type(type)) {
        return static_cast<double>(va_arg(*args, unsigned int));
    }
    if (name == "long") {
        return static_cast<double>(va_arg(*args, long));
    }
    if (name == "unsigned long") {
        return static_cast<double>(va_arg(*args, unsigned long));
    }
    if (name == "long long") {
        return static_cast<double>(va_arg(*args, long long));
    }
    if (name == "unsigned long long") {
        return static_cast<double>(va_arg(*args, unsigned long long));
    }
    return static_cast<double>(va_arg(*args, int));
}

static std::string storage_dir() {
    const char* dir = std::getenv(kStorageDirEnv);
    return (dir != nullptr && dir[0] != '\0') ? dir : kDefaultStorageDir;
}

static std::string sanitize_file_name(const std::string& value) {
    std::string out;
    for (char ch : value) {
        const bool allowed =
            (ch >= 'a' && ch <= 'z') ||
            (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9') ||
            ch == '-' || ch == '_';
        out.push_back(allowed ? ch : '_');
    }
    return out.empty() ? "_empty_" : out;
}

static std::string data_file_path(const std::string& scope_id) {
    return storage_dir() + "/" + sanitize_file_name(scope_id) + ".jsonl";
}

static void ensure_storage_dir_exists() {
    mkdir(storage_dir().c_str(), 0777);
}

static void json_string(std::ostringstream& out, const std::string& value) {
    out << '"';
    for (char ch : value) {
        switch (ch) {
            case '"':
            case '\\': out << '\\' << ch; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << ch; break;
        }
    }
    out << '"';
}

static std::string to_json_line(const std::string& scope_id,
                                const CapturedPoint& point,
                                unsigned long long step) {
    std::ostringstream out;
    out << "{\"scope_id\":";
    json_string(out, scope_id);
    out << ",\"key\":";
    json_string(out, scope_id + "_" + point.port);
    out << ",\"port\":";
    json_string(out, point.port);
    out << ",\"type\":";
    json_string(out, point.type);
    out << ",\"value\":{\"x\":" << step << ",\"y\":" << point.y << "}";
    out << ",\"sequence\":" << (step + 1) << "}";
    return out.str();
}

static unsigned long long next_step(const std::string& scope_id) {
    ensure_storage_dir_exists();
    if (g_started_scopes.insert(scope_id).second) {
        std::ofstream(data_file_path(scope_id), std::ios::trunc);
        g_next_step_by_scope[scope_id] = 0;
    }
    return g_next_step_by_scope[scope_id]++;
}

static std::vector<CapturedPoint> read_points(va_list* args) {
    std::vector<CapturedPoint> points;

    const char* port = va_arg(*args, const char*);
    while (!is_end_marker(port)) {
        const char* type = va_arg(*args, const char*);
        if (is_end_marker(type)) {
            break;
        }
        if (!is_supported_type(type)) {
            port = type;
            continue;
        }

        CapturedPoint point;
        point.port = port;
        point.type = type;
        point.y = read_value(args, type);
        points.push_back(point);

        port = va_arg(*args, const char*);
    }

    return points;
}

} // namespace

extern "C" void scope_push_send_v(const char* scope_id, ...) {
    if (!scope_id) return;

    va_list ap;
    va_start(ap, scope_id);
    std::vector<CapturedPoint> points = read_points(&ap);
    va_end(ap);

    if (points.empty()) return;

    std::lock_guard<std::mutex> lock(g_mutex);
    const std::string scope(scope_id);
    const unsigned long long step = next_step(scope);

    std::ostringstream batch;
    for (const auto& point : points) {
        batch << to_json_line(scope, point, step) << '\n';
    }

    std::ofstream out(data_file_path(scope), std::ios::app);
    if (!out.is_open()) return;
    out << batch.str();
}

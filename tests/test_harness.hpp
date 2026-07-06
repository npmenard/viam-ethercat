#pragma once

// Tiny header-only test harness. No GoogleTest: each test file #includes this,
// declares cases with TEST(name) { ... }, and ends with TEST_MAIN(). One
// executable per test file, wired into ctest via add_test. A test fails by
// throwing (CHECK macros throw); the runner reports per-case PASS/FAIL and
// returns non-zero if any case failed.

#include <cstdio>
#include <exception>
#include <functional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace etest {

struct Failure {
    std::string message;
};

struct Case {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

inline int run_all() {
    int failed = 0;
    for (const auto& test_case : registry()) {
        try {
            test_case.fn();
            std::printf("[ PASS ] %s\n", test_case.name);
        } catch (const Failure& failure) {
            std::printf("[ FAIL ] %s\n         %s\n", test_case.name, failure.message.c_str());
            ++failed;
        } catch (const std::exception& ex) {
            std::printf("[ FAIL ] %s\n         unexpected exception: %s\n", test_case.name, ex.what());
            ++failed;
        } catch (...) {
            std::printf("[ FAIL ] %s\n         unexpected non-std exception\n", test_case.name);
            ++failed;
        }
    }
    const auto total = static_cast<int>(registry().size());
    std::printf("---- %d test(s), %d failed ----\n", total, failed);
    return failed == 0 ? 0 : 1;
}

// Build a Failure with a "file:line PREFIX: detail" message.
template <class... Parts>
[[noreturn]] inline void fail(const char* file, int line, const std::string& detail) {
    std::ostringstream os;
    os << file << ':' << line << ' ' << detail;
    throw Failure{os.str()};
}

// Stream a value for diagnostics, coping with types that don't stream nicely:
// scoped enums (print the underlying value) and char-like 8-bit ints (print as
// a number, not a control character).
template <class T>
void stream_value(std::ostream& os, const T& value) {
    if constexpr (std::is_enum_v<T>) {
        os << static_cast<long long>(value);
    } else if constexpr (std::is_same_v<T, char> || std::is_same_v<T, signed char> || std::is_same_v<T, unsigned char>) {
        os << static_cast<int>(value);
    } else {
        os << value;
    }
}

template <class A, class B>
std::string format_eq(const std::string& expr, const A& got, const B& expected) {
    std::ostringstream os;
    os << expr << "  (got ";
    stream_value(os, got);
    os << ", expected ";
    stream_value(os, expected);
    os << ')';
    return os.str();
}

}  // namespace etest

#define ETEST_CONCAT_INNER(a, b) a##b
#define ETEST_CONCAT(a, b) ETEST_CONCAT_INNER(a, b)

#define TEST(human_name)                                                                                                  \
    static void ETEST_CONCAT(etest_case_, __LINE__)();                                                                    \
    static const ::etest::Registrar ETEST_CONCAT(etest_reg_, __LINE__){human_name, &ETEST_CONCAT(etest_case_, __LINE__)}; \
    static void ETEST_CONCAT(etest_case_, __LINE__)()

// Boolean assertion.
#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            ::etest::fail(__FILE__, __LINE__, std::string("CHECK failed: ") + #cond); \
        }                                                                             \
    } while (0)

// Equality assertion that reports both operands.
#define CHECK_EQ(got, expected)                                                                                      \
    do {                                                                                                             \
        const auto etest_got = (got);                                                                                \
        const auto etest_expected = (expected);                                                                      \
        if (!(etest_got == etest_expected)) {                                                                        \
            ::etest::fail(__FILE__, __LINE__, ::etest::format_eq(#got " == " #expected, etest_got, etest_expected)); \
        }                                                                                                            \
    } while (0)

// Assert that evaluating expr throws an exception of (at least) type ExType.
#define CHECK_THROWS(expr, ExType)                                                                 \
    do {                                                                                           \
        bool etest_threw = false;                                                                  \
        try {                                                                                      \
            (void)(expr);                                                                          \
        } catch (const ExType&) {                                                                  \
            etest_threw = true;                                                                    \
        }                                                                                          \
        if (!etest_threw) {                                                                        \
            ::etest::fail(__FILE__, __LINE__, std::string("expected " #ExType " from: ") + #expr); \
        }                                                                                          \
    } while (0)

// Assert that evaluating expr throws ExType whose what() CONTAINS `needle`.
#define CHECK_THROWS_MSG(expr, ExType, needle)                                                                                   \
    do {                                                                                                                         \
        bool etest_ok = false;                                                                                                   \
        try {                                                                                                                    \
            (void)(expr);                                                                                                        \
        } catch (const ExType& etest_ex) {                                                                                       \
            etest_ok = std::string_view(etest_ex.what()).find(needle) != std::string_view::npos;                                 \
        }                                                                                                                        \
        if (!etest_ok) {                                                                                                         \
            ::etest::fail(__FILE__, __LINE__, std::string("expected " #ExType " containing \"") + (needle) + "\" from: " #expr); \
        }                                                                                                                        \
    } while (0)

#define TEST_MAIN()                \
    int main() {                   \
        return ::etest::run_all(); \
    }

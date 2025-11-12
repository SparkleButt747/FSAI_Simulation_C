#pragma once

#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace testing {

struct TestInfo {
    std::string name;
    void (*func)();
};

inline std::vector<TestInfo>& Registry() {
    static std::vector<TestInfo> tests;
    return tests;
}

struct TestRegistrar {
    TestRegistrar(const char* suite, const char* name, void (*func)()) {
        Registry().push_back({std::string(suite) + "." + name, func});
    }
};

inline bool& FailureFlag() {
    static bool flag = false;
    return flag;
}

inline void ResetFailure() {
    FailureFlag() = false;
}

inline void Fail() {
    FailureFlag() = true;
}

inline bool CurrentTestFailed() {
    return FailureFlag();
}

inline void LogFailure(const char* file, int line, const std::string& message) {
    Fail();
    std::cerr << file << ":" << line << " " << message << std::endl;
}

inline int InitGoogleTest(int*, char**) {
    return 0;
}

inline int RunAllTests() {
    int failed = 0;
    for (const auto& test : Registry()) {
        std::cout << "[ RUN      ] " << test.name << std::endl;
        ResetFailure();
        test.func();
        if (CurrentTestFailed()) {
            ++failed;
            std::cout << "[  FAILED  ] " << test.name << std::endl;
        } else {
            std::cout << "[       OK ] " << test.name << std::endl;
        }
    }
    return failed == 0 ? 0 : 1;
}

}  // namespace testing

#define TEST(SuiteName, TestName)                                                          \
    void SuiteName##_##TestName##_Test();                                                   \
    static ::testing::TestRegistrar SuiteName##_##TestName##_registrar(                     \
        #SuiteName, #TestName, &SuiteName##_##TestName##_Test);                             \
    void SuiteName##_##TestName##_Test()

#define EXPECT_TRUE(condition)                                                              \
    do {                                                                                    \
        if (!(condition)) {                                                                 \
            ::testing::LogFailure(__FILE__, __LINE__,                                       \
                                 std::string("Expected true: ") + #condition);             \
        }                                                                                   \
    } while (0)

#define EXPECT_FALSE(condition)                                                             \
    do {                                                                                    \
        if (condition) {                                                                    \
            ::testing::LogFailure(__FILE__, __LINE__,                                       \
                                 std::string("Expected false: ") + #condition);            \
        }                                                                                   \
    } while (0)

#define EXPECT_EQ(val1, val2)                                                               \
    do {                                                                                    \
        auto _expected = (val1);                                                            \
        auto _actual = (val2);                                                              \
        if (!(_expected == _actual)) {                                                      \
            std::ostringstream _oss;                                                        \
            _oss << "Expected equality of " << #val1 << " and " << #val2                  \
                 << ", but got " << _expected << " and " << _actual;                     \
            ::testing::LogFailure(__FILE__, __LINE__, _oss.str());                          \
        }                                                                                   \
    } while (0)

#define RUN_ALL_TESTS() ::testing::RunAllTests()


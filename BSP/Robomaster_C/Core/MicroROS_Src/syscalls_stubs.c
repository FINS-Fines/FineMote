// --- START OF FILE Core/Src/syscalls_stubs.c ---

#include <stddef.h> // for NULL

// =========================================================================
// 提供 micro-ROS 预编译库所需的 POSIX 和标准 C 库函数的桩实现
// 以解决链接时出现的 "Undefined symbol" 错误。
// =========================================================================


// --- 1. 环境变量函数 (POSIX) ---
// 在裸机/RTOS环境中，环境变量没有意义，所以提供空实现。
int setenv(const char *name, const char *value, int overwrite) {
    (void)name;
    (void)value;
    (void)overwrite;
    return 0; // 总是返回成功
}

int unsetenv(const char *name) {
    (void)name;
    return 0; // 总是返回成功
}


// --- 2. ctype 内部变量 ---
// _ctype_ 是一个用于 ctype.h 函数（如 isalpha）的查找表。
// 我们可以提供一个弱符号的空数组来满足链接器。
// 或者，如果您的工具链有其他方法，但这通常是最简单的。
// 注意：这可能意味着某些 ctype 函数行为不正确，但对于 topic name 验证通常足够。
const char _ctype_[256] = {0};


// --- 3. 可重入性支持 (Newlib) ---
// _impure_ptr 是 Newlib C 库用于线程安全的内部变量。
// ARM C 库可能不使用它，但 micro-ROS 的某些部分（特别是如果它是用GCC/Newlib编译的）
// 可能会引用它。我们提供一个空指针。
void * _impure_ptr = NULL;

// --- END OF FILE Core/Src/syscalls_stubs.c ---
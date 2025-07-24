#include <iostream>
#include <type_traits>

// 通用模板
template <class T, class Enable = void>
class Test{
public:
    explicit Test(T a) : a_(a) {
        std::cout << "普通模板化: " << a_ << std::endl;
    }
};

// 针对浮点类型的偏特化
template <class T>
class Test<T, typename std::enable_if<std::is_floating_point<T>::value>::type> {
public:
    explicit Test(T a) : a_(a) {
        std::cout << "double 偏特化" << a_ << std::endl;
    }

private:
    T a_;
};

/**
 * 举例
 * std::enable_if<std::is_floating_point<double>::value>::type
 * - std::is_floating_point<double>::value 是 true
 * - 所以 std::enable_if<true>::type 就是 void
 * std::enable_if<std::is_floating_point<int>::value>::type
 * - std::is_floating_point<int>::value 是 false
 * - 所以 std::enable_if<false>::type 没有定义，导致模板特化失效
 */

/**
 * 如果你想让 type 返回别的类型，可以写成 std::enable_if<条件, 你想要的类型>::type，比如
 * std::enable_if<std::is_floating_point<T>::value, int>::type
 * 这时如果条件为 true，type 就是 int。
 */

/**
 * 提问：typename std::enable_if<std::is_floating_point<T>::value>::type 实际上等价于 typename void 吗？
 * 回答：不是的。
 * std::enable_if<条件>::type，当条件为 true 时，type 是 void（默认），否则没有 type 这个成员。
 * typename 只是告诉编译器，std::enable_if<...>::type 是一个类型（不是变量、不是值）。
 * typename void 其实没有意义，因为 void 已经是一个类型了，不需要 typename 修饰。
 * typename 只能用于依赖于模板参数的类型名（dependent type name），比如 typename T::value_type。
 * 直接写 typename void，编译器会报错。
 */

/**
* 提问：为什么 typename std::enable_if<...>::type 要加 typename？
* 回答：因为 std::enable_if<...>::type 依赖于模板参数 T，编译器需要 typename 来区分它是类型，不是静态成员或变量。
* 
* 例子：
* // 正确
* typename std::enable_if<std::is_floating_point<T>::value>::type
* 
* // 错误
* typename void // 编译报错
*/
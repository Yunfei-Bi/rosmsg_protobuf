**如果我在cmakelists中也指定了install()的路径，那么最终哪个路径会生效？
cmakelists中的install()还是 -DCMAKE_INSTALL_PREFIX**

CMake 中 install() 与 -DCMAKE_INSTALL_PREFIX 的优先级关系
这是一个常见的 CMake 配置问题。当你在 CMakeLists.txt 中使用 install() 命令指定路径，同时又通过 -DCMAKE_INSTALL_PREFIX 参数设置安装前缀时，两者会协同工作，而非互相覆盖。

**执行机制**
- -DCMAKE_INSTALL_PREFIX 的作用
- - 设置全局安装前缀（base directory），默认值为 /usr/local。
- - 所有 install() 命令的相对路径都会**基于这个前缀展开**。

- install() 命令的路径格式
- - 支持绝对路径和相对路径：
```cmake
# 相对路径（相对于 CMAKE_INSTALL_PREFIX）
install(TARGETS mylib DESTINATION lib)  # 安装到 ${CMAKE_INSTALL_PREFIX}/lib

# 绝对路径（忽略 CMAKE_INSTALL_PREFIX）
install(FILES config.txt DESTINATION /etc/myapp)  # 直接安装到 /etc/myapp
```
**优先级规则**
- 相对路径：install() 的路径**会被追加到** CMAKE_INSTALL_PREFIX 后面。
例如：
```cmake
# CMakeLists.txt
install(TARGETS mylib DESTINATION lib)  # 相对路径

# 命令行
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/myapp
→ 实际安装路径：/opt/myapp/lib
```
- 绝对路径：install() 的路径会**直接覆盖** CMAKE_INSTALL_PREFIX。
例如：
```cmake
# CMakeLists.txt
install(TARGETS mylib DESTINATION /usr/lib)  # 绝对路径

# 命令行
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/myapp  # 此设置被忽略
```

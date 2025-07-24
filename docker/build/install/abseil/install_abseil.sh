#1usr/bin/env bash

# 当脚本执行到一个返回非零状态码的命令时：
# 如果没有 set -e，脚本会继续执行后续命令
# 如果设置了 set -e，脚本会立即终止并退出，不再执行后续命令

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

#https://github.com/abseil/abseil-cpp/archive/refs/tags/20230802.0.tar.gz
# Install abseil
THREAD_NUM=$(nproc)
VERSION="20230802.0"
PKG_NAME="abseil-cpp-${VERSION}.tar.gz"

tar xzf "${PKG_NAME}"
pushd "abseil-cpp-${VERSION}"
    mkdir build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_CXX_STANDARD=14 \
        -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j${THREAD_NUM}
    make install
popd

# ldconfig 是 Linux 系统中用于更新动态链接库缓存的命令，
# 在安装或更新共享库后必须执行，以确保系统能够正确找到并加载这些库。

ldconfig

# Clean up
rm -rf "abseil-cpp-${VERSION}" "PKG_NAME"

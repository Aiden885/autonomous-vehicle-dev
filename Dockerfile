FROM ubuntu:20.04

# 避免交互式安装卡住
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# ─── 1. 基础工具 ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    pkg-config \
    autoconf \
    automake \
    libtool \
    unzip \
    vim \
    gdb \
    valgrind \
    && rm -rf /var/lib/apt/lists/*

# ─── 2. 项目依赖库（apt 可以干净安装） ──────────────────────
RUN apt-get update && apt-get install -y \
    # ZMQ
    libzmq3-dev \
    # OpenCV 4.2
    libopencv-dev \
    # Eigen3
    libeigen3-dev \
    # Qt5
    qtbase5-dev \
    # yaml-cpp
    libyaml-cpp-dev \
    # libxml2
    libxml2-dev \
    # pthread（glibc 自带，但确保 -lpthread 可用）
    # Boost（date_time、timer）
    libboost-date-time-dev \
    libboost-timer-dev \
    # 其他常用
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# ─── 3. Protobuf C++ 3.6.1（从项目自带源码包编译） ──────────
COPY protobuf-all-3.6.1.gz /tmp/
RUN cd /tmp && \
    tar -xzf protobuf-all-3.6.1.gz && \
    cd protobuf-3.6.1 && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp && rm -rf protobuf-3.6.1 protobuf-all-3.6.1.gz

# ─── 4. Protobuf-C 1.3.3（C 语言版，从项目自带源码包编译） ──
COPY protobuf-c-1.3.3.gz /tmp/
RUN cd /tmp && \
    tar -xzf protobuf-c-1.3.3.gz && \
    cd protobuf-c-1.3.3 && \
    ./configure --prefix=/usr/local PKG_CONFIG_PATH=/usr/local/lib/pkgconfig && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp && rm -rf protobuf-c-1.3.3 protobuf-c-1.3.3.gz

# ─── 5. 环境变量（让 cmake 找到 /usr/local 下的库） ─────────
ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ENV PATH=/usr/local/bin:$PATH

# ─── 6. 工作目录 ─────────────────────────────────────────────
WORKDIR /workspace

# 验证关键工具版本
RUN echo "=== 环境验证 ===" && \
    gcc --version | head -1 && \
    g++ --version | head -1 && \
    cmake --version | head -1 && \
    protoc --version && \
    protoc-c --version && \
    pkg-config --modversion libzmq && \
    pkg-config --modversion opencv4 && \
    pkg-config --modversion eigen3 && \
    echo "=== 环境配置完成 ==="

CMD ["/bin/bash"]

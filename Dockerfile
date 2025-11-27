FROM ubuntu:plucky AS builder
# Configure apt caching
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Update package list
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt update

# Install core build tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    build-essential \
    bison \
    ccache \
    debhelper \
    flex \
    meson \
    ninja-build \
    pkgconf \
    python3 \
    python3-venv

# Install Python documentation tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    python3-sphinx \
    python3-sphinx-rtd-theme

# Install core libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libglib2.0-dev \
    libpixman-1-dev \
    zlib1g-dev

# Install compression and security libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libcap-ng-dev \
    libseccomp-dev \
    libzstd-dev \
    nettle-dev \
    libgnutls28-dev \
    libsasl2-dev

# Install image format libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libjpeg-dev \
    libjpeg-turbo8-dev \
    libpng-dev

# Install graphics and display libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libdrm-dev \
    libepoxy-dev \
    libgbm-dev \
    libgtk-3-dev \
    libsdl2-dev \
    libvirglrenderer-dev \
    libvte-2.91-dev

# Install audio libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libasound2-dev \
    libjack-dev \
    libpipewire-0.3-dev \
    libpulse-dev

# Install storage and block device libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libaio-dev \
    libblkio-dev \
    libfuse3-dev \
    libglusterfs-dev \
    libiscsi-dev \
    libnfs-dev \
    librbd-dev \
    liburing-dev

# Install network and protocol libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libcurl4-gnutls-dev \
    libslirp-dev \
    libssh-dev

# Install RDMA and InfiniBand libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libibumad-dev \
    libibverbs-dev \
    librdmacm-dev

# Install USB and device libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libudev-dev \
    libusb-1.0-0-dev \
    libusbredirparser-dev

# Install smartcard and virtualization libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libcacard-dev \
    libspice-server-dev

# Install miscellaneous device support libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libbpf-dev \
    libbrlapi-dev \
    libfdt-dev \
    libncurses-dev \
    libnuma-dev \
    libpmem-dev

# Install testing libraries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt install -y \
    libcmocka-dev \
    libjson-c-dev

# Copy build files
COPY Makefile Makefile
COPY patches patches
COPY qemu qemu

# Build with ccache enabled
RUN --mount=type=cache,target=/root/.ccache \
    export PATH="/usr/lib/ccache:$PATH" && \
    export CC="ccache gcc" && \
    export CXX="ccache g++" && \
    make -j`nproc`

# Copy build artifact
RUN cp ./build-*/qemu-build-*.tar.gz /qemu-build.tar.gz

FROM ubuntu:plucky

# Copy tarball from builder stage
COPY --from=builder /qemu-build.tar.gz /tmp/build.tar.gz

# Extract tarball to system root
RUN tar -xzf /tmp/build.tar.gz -C / && rm /tmp/build.tar.gz

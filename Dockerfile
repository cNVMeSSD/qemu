FROM ubuntu:plucky

RUN apt update && apt install -y \
    bison libbrlapi-dev libgbm-dev libjack-dev libpmem-dev \
    libslirp-dev libvte-2.91-dev python3-sphinx-rtd-theme build-essential libcacard-dev \
    libglib2.0-dev libjpeg-dev libpng-dev libspice-server-dev libzstd-dev python3-venv \
    debhelper libcap-ng-dev libglusterfs-dev libjpeg-turbo8-dev libpulse-dev libssh-dev \
    meson zlib1g-dev flex libcurl4-gnutls-dev libgnutls28-dev libncurses-dev \
    librbd-dev libudev-dev nettle-dev libaio-dev libdrm-dev libgtk-3-dev \
    libnfs-dev librdmacm-dev liburing-dev ninja-build libasound2-dev libepoxy-dev \
    libibumad-dev libnuma-dev libsasl2-dev libusb-1.0-0-dev pkgconf \
    libblkio-dev libfdt-dev libibverbs-dev libpipewire-0.3-dev libsdl2-dev \
    libusbredirparser-dev python3 libbpf-dev libfuse3-dev libiscsi-dev libpixman-1-dev \
    libseccomp-dev libvirglrenderer-dev python3-sphinx libjson-c-dev libcmocka-dev

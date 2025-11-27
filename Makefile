VER = cnvmessd
BUILD_DIR = build-$(VER)
INST_DIR = $(BUILD_DIR)/inst
CONFIGURE_OPTS = \
	--enable-linux-aio \
	--prefix=/opt/qemu-$(VER) \
	--disable-xen \
	--disable-sdl \
	--disable-gtk \
	--enable-kvm \
	--localstatedir='/var' \
	--disable-user \
	--disable-linux-user \
	--disable-vnc \
	--enable-vfio-user-server \
	--static

.PHONY: all clean configure build install package help

all: package

help:
	@echo "Available targets:"
	@echo "  all        - Build and package QEMU (default)"
	@echo "  configure  - Configure the build"
	@echo "  build      - Build QEMU"
	@echo "  install    - Install QEMU to staging directory"
	@echo "  package    - Create distribution tarball"
	@echo "  clean      - Remove build directory"
	@echo "  help       - Show this help message"

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

configure: $(BUILD_DIR)
	cd $(BUILD_DIR) && ../qemu/configure $(CONFIGURE_OPTS)

build: configure
	$(MAKE) -C $(BUILD_DIR) -j$(shell nproc)

install: build
	rm -rf $(INST_DIR)
	mkdir -p $(INST_DIR)
	$(MAKE) -C $(BUILD_DIR) install DESTDIR=$(CURDIR)/$(INST_DIR)

package: install
	cd $(INST_DIR) && \
	ln -sf qemu-$(VER) opt/qemu && \
	rm -f ../qemu-build-*.tar.gz && \
	tar czvf ../qemu-build-$(VER).tar.gz * --exclude=var --owner=0 --group=0

clean:
	rm -rf $(BUILD_DIR)

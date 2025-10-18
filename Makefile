QEMU_TARBALL := qemu-v10.1.1.tar.gz

extract:
	mkdir -p ${CURDIR}/qemu && tar -xf ${CURDIR}/${QEMU_TARBALL} -C ${CURDIR}/qemu --strip-components=1

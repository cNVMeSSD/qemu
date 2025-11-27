# QEMU PCI-Express Endpoint Controller

This repository contains a set of files and patches to add an endpoint
controller to QEMU. It is based on QEMU version v10.1 as this has the
vfio-user client upstreamed meaning we can use it for both the provider
of the PCI-Express endpoint by providing a vfio-user server and the user
of the PCI-Express endpoint by using the upstreamed vfio-user client.

## Building

Only Linux systems are supported at this moment, on a Linux system, just
run `make` and you'll get a QEMU binary. The only necessary dependencies
are the dependencies of QEMU.

## Updating QEMU

The changes made to QEMU are tracked in the patchset in the `patches`
directory and are in the git tree. Use the `quiltrc` file in `patches`
to ensure correct settings:

```bash
export QUILTRC=patches/quiltrc
```

To rebase, pop the patches using `quilt pop -a` and pull in new QEMU sources
into the `qemu` directory using `git subtree`:

```bash
quilt pop -a
git subtree pull --prefix qemu https://gitlab.com/qemu-project/qemu.git v10.1.2 --squash
```

Then, re-apply the patches, and amend them where necessary:

```bash
quilt push -a
```

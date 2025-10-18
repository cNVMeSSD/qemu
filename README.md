# QEMU PCI-Express Endpoint Controller

This repository contains a set of files and patches to add an endpoint
controller to QEMU. It is based on QEMU version v10.1 as this has the
vfio-user client upstreamed meaning we can use it for both the provider
of the PCI-Express endpoint by providing a vfio-user server and the user
of the PCI-Express endpoint by using the upstreamed vfio-user client.

## Building

Only Linux systems are supported at this moment, on a Linux system, just
run `make` and you'll get a QEMU binary.

## Updating QEMU

Get yourself a new tarball, and change the source tarball in the `Makefile`.
You'll probably have to update the patches, you can use `quilt` to apply and
track these, as the `qemu` folder is not tracked in `git`.

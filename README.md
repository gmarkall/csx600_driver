ClearSpeed CSX600 Driver for Linux Kernel 2.6.24
==================================================

This is a modified version of the ClearSpeed CSX600 driver for kernel version 2.6.24.

I've tried this with an X620 card, and csdiag gives the following output:

```
$ ./csdiag

Time: Tue Apr 27 18:53:56 2010
Host: xeon1 Linux x86_64 2.6.24-24-server
SVer: 3.11 (1.404.1.39 at Thu Aug 21 16:50:25 BST 2008 on linux_x86_64)
Card: Instance 0, Bus 6, Device 1 (1 card present)
X620 (PCI-X), FPGA 0x3f020000, SN CLSJ06110026

Basic checks: PASS
Reset card: PASS
Temperature: PASS
Frequency: PASS
PCI Read BW: PASS
PCI Write BW: PASS
Combined BW: PASS
Memory tests: PASS
Semaphores: PASS
PIO tests: PASS
PE memory: PASS
Compute test: PASS
Temperature: PASS
Frequency: PASS

Overall result: PASS
```

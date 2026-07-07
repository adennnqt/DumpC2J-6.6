/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compatibility shim for kernel < 6.12, which only has asm/unaligned.h.
 * Upstream Linux renamed asm/unaligned.h -> linux/unaligned.h in v6.12
 * (commit 5f60d5f6bbc1). This shim lets newer vendored code (e.g. zstd
 * synced from Linux 6.15) that expects <linux/unaligned.h> keep working
 * on this older kernel base without modifying any existing file.
 */
#ifndef _LINUX_UNALIGNED_COMPAT_SHIM_H
#define _LINUX_UNALIGNED_COMPAT_SHIM_H

#include <asm/unaligned.h>

#endif

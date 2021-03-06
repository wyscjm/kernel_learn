/*
 * Freescale STMP378X PxP driver
 *
 * Author: Matt Porter <mporter@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008-2009 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

struct pxp_overlay_registers {
	u32 ol;
	u32 olsize;
	u32 olparam;
	u32 olparam2;
};

/* Registers feed for PXP_NEXT */
struct pxp_registers {
	u32 ctrl;
	u32 rgbbuf;
	u32 rgbbuf2;
	u32 rgbsize;
	u32 s0buf;
	u32 s0ubuf;
	u32 s0vbuf;
	u32 s0param;
	u32 s0background;
	u32 s0crop;
	u32 s0scale;
	u32 s0offset;
	u32 s0colorkeylow;
	u32 s0colorkeyhigh;
	u32 olcolorkeylow;
	u32 olcolorkeyhigh;

	struct pxp_overlay_registers ol0;
	struct pxp_overlay_registers ol1;
	struct pxp_overlay_registers ol2;
	struct pxp_overlay_registers ol3;
	struct pxp_overlay_registers ol4;
	struct pxp_overlay_registers ol5;
	struct pxp_overlay_registers ol6;
	struct pxp_overlay_registers ol7;
};

struct pxp_buffer {
	/* Must be first! */
	struct videobuf_buffer vb;
	struct list_head queue;
};

struct pxps {
	struct platform_device *pdev;
	struct resource *res;
	int irq;
	void __iomem *regs;

	struct work_struct work;
	struct workqueue_struct *workqueue;
	spinlock_t lock;
	struct mutex mutex;
	int users;

	struct video_device *vdev;

	struct videobuf_queue s0_vbq;
	struct videobuf_buffer *active;
	struct list_head outq;
	struct list_head nextq;

	int output;
	u32 *outb;
	dma_addr_t outb_phys;

	/* Current S0 configuration */
	struct pxp_data_format *s0_fmt;
	u32 s0_width;
	u32 s0_height;
	u32 s0_bgcolor;
	u32 s0_chromakey;

	struct v4l2_framebuffer fb;
	struct v4l2_rect drect;
	struct v4l2_rect srect;

	/* Transformation support */
	int scaling;
	int hflip;
	int vflip;
	int rotate;
	int yuv;

	/* Output overlay support */
	int overlay_state;
	int global_alpha_state;
	u8  global_alpha;
	int local_alpha_state;
	int s1_chromakey_state;
	u32 s1_chromakey;

	/* PXP_NEXT */
	u32 regs_phys;
	struct pxp_registers *regs_virt;
	wait_queue_head_t done;
	int next_queue_ended;
};

struct pxp_data_format {
	char *name;
	unsigned int bpp;
	u32 fourcc;
	enum v4l2_colorspace colorspace;
	u32 ctrl_s0_fmt;
};

extern int stmp3xxxfb_get_info(struct fb_var_screeninfo *var,
				struct fb_fix_screeninfo *fix);
extern void stmp3xxxfb_cfg_pxp(int enable, dma_addr_t pxp_phys);
